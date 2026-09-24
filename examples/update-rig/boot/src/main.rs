//! BOOT-role firmware for the A/B update rig on the ATSAMD51J20A:
//! it validates the fuses, waits on the link whenever the application
//! has asked for an update, and streams an image off the wire straight into
//! the inactive bank.
//!
//! # Flow
//!
//! 1. The link comes up first, because it also installs the `rprintln!`
//!    sink (RTT by default, SERCOM5 at 115200 under `--features rs485`).
//! 2. [`Boot::new`] validates the fuses (provisioned off-board); an
//!    unprovisioned part parks with a message instead of running any of
//!    this.
//! 3. [`Boot::boot_or_enter_download`] takes the boot decision, and a jump
//!    into the active image goes through verification first. It returns
//!    for an update request, when nothing on the part is known to boot, or
//!    when a store write failed. There is no listening window, so an
//!    update is always something the application asked for before it
//!    reset.
//! 4. BOOT then waits for a [`Message::BeginUpdate`] with no deadline and
//!    pulls exactly `len` raw bytes off the wire into [`Boot::install`],
//!    which verifies the image, records the trial, and swaps banks. The
//!    swap reboots, so a successful install never returns; a failed one
//!    comes back and is reported as [`Message::UpdateResult`].
//! 5. A failed install holds the link open until the host speaks again,
//!    then the loop returns to step 3: a rejected update must not cost the
//!    device its working app.
//!
//! # The image never lands in RAM
//!
//! `ImageStream` is an `Iterator<Item = u8>` that blocking-reads one byte
//! per `next`, so samd5-boot's writer pulls the image through a single
//! flash page buffer at a time. RAM cost is that page, whatever the image
//! size.
//!
//! # Why the receive path is buffered
//!
//! The protocol defines no ack inside the image body, so nothing throttles
//! the host mid-image, while the writer stops reading for an NVMCTRL program
//! at every page and for an erase at every 16-page block. The transport has
//! to absorb that; see `RX_RING` in `demo-serial`.
#![no_std]
#![no_main]

use atsamd_hal as hal;
use cortex_m::peripheral::{SYST, syst::SystClkSource};
use hal::watchdog::{Watchdog, WatchdogTimeout};
use proto::{Decoder, Feed, Message, Status};
use rtt_target::rprintln;
use samd5_boot::{
    Aborted, Boot, BootConfig, FlashError, InstallError, Unverified,
    boot_info::{self, BootInfo},
    consts, install_boot_info,
    persist::{BkupRamStore, BootStorage},
};

// The transport is whichever crate the `rtt` / `rs485` feature selected.
// Both expose the same `Link` surface; only the constructor differs, which
// is the one thing cfg'd at the call site below.
#[cfg(feature = "rs485")]
use demo_serial as link;
#[cfg(feature = "rtt")]
use demo_rtt as link;
use link::Link;

/// Silence that abandons a raw image body mid-stream.
const BYTE_TIMEOUT_MS: u32 = 1000;
/// How long BOOT holds the link open after reporting a failed install, so
/// the host can read the report before the next image overwrites it.
const HANDOVER_TIMEOUT_MS: u32 = 2000;

install_boot_info!(BootInfo {
    magic: boot_info::MAGIC,
    abi_version: boot_info::ABI_VERSION,
    // The `proto` wire protocol this BOOT speaks.
    transport_version: 1,
    boot_size: consts::BOOT_SIZE as u32,
    build_id: 0,
});

#[cortex_m_rt::entry]
fn main() -> ! {
    // Only the RS485 constructor borrows these mutably.
    #[cfg_attr(feature = "rtt", allow(unused_mut))]
    let mut p = hal::pac::Peripherals::take().expect("Infallible");
    let core = cortex_m::Peripherals::take().expect("Infallible");

    // Bringing the link up first also sets up `rprintln!`: over RTT the two
    // share one control block, so the transport owns printing.
    #[cfg(feature = "rtt")]
    let mut serial = Link::new();
    #[cfg(feature = "rs485")]
    let mut serial = {
        rtt_target::rtt_init_print!();
        let pins = hal::gpio::Pins::new(p.port);
        Link::new(
            p.sercom5,
            &mut p.mclk,
            &mut p.gclk,
            pins.pb02,
            pins.pb03,
            pins.pb00,
        )
    };
    rprintln!("boot: start");

    let nvm = hal::nvm::Nvm::new(p.nvmctrl);
    let Ok(dsu) = hal::dsu::Dsu::new(p.dsu, &p.pac) else {
        rprintln!("boot: DSU unavailable");
        park()
    };
    let config = BootConfig {
        max_boot_attempts: 3,
        trial_timeout: WatchdogTimeout::Cycles16K,
    };
    let Ok(boot) = Boot::new(nvm, dsu, Watchdog::new(p.wdt), config) else {
        rprintln!("boot: fuses not provisioned, run `cargo xtask provision`");
        park()
    };

    let mut clock = Millis::new(core.SYST);
    let mut decoder = Decoder::new();
    // Backup RAM survives the BKSWRST reset, so trial bookkeeping works,
    // while a power cut reads back as a fresh store rather than a stale
    // trial. It also keeps the rig off the SmartEEPROM fuses.
    // SAFETY: nothing else in this rig uses the base of backup RAM.
    let mut store = unsafe { BkupRamStore::<{ link::STORE_OFFSET }>::new() };

    let mut boot = boot;
    loop {
        // The receive interrupt must not outlive this image: past this call
        // the core may be running the application, vectoring through its
        // table.
        serial.quiesce();
        boot = boot.boot_or_enter_download(&mut store);
        serial.arm();

        rprintln!("boot: waiting for an image (no deadline)");
        let len = listen(&mut serial, &mut decoder);
        boot = install(boot, &mut store, &mut serial, &mut clock, len);
        // The device should go back into service rather than sit here: a
        // valid image, if there is one, is better than an indefinite wait.
        // The host asks again through the application to retry.
        //
        // But not before the host has actually taken the failure report.
        // On a buffered transport the reply is still in RAM at this point,
        // and the application that boots next zeroes .bss on its way up,
        // which over RTT is the very buffer holding it. A lost report reads
        // to the host exactly like a successful install.
        handover(&mut serial, &mut clock);
    }
}

/// Wait for a `BeginUpdate`, answering pings on the way. There is no
/// deadline: a fixed window would be a race the host has to win, and BOOT
/// only gets here once the application has asked for an update, or once
/// nothing bootable is left.
fn listen(serial: &mut Link, decoder: &mut Decoder) -> u32 {
    loop {
        // One byte per feed, so a completed frame never carries a tail:
        // whatever follows the COBS delimiter (the raw image body, after
        // a BeginUpdate) is still in the SERCOM for `ImageStream`.
        if let Some(b) = serial.try_read_byte() {
            match decoder.feed(&[b]) {
                Feed::Frame { msg, .. } => match msg {
                    Message::BeginUpdate { len } => return len,
                    Message::Ping(payload) => send(serial, &Message::Pong(payload)),
                    _ => rprintln!("boot: ignoring a message that is not for BOOT"),
                },
                Feed::DeserError { .. } | Feed::Overfull { .. } => rprintln!("boot: bad frame"),
                Feed::Consumed => (),
            }
        }
    }
}

/// Wait for the host to show it has read what was just sent, so the reply
/// is not still in a buffer when the next image clears it.
///
/// Any inbound byte will do: the host only speaks again once it has taken
/// the reply. The timeout is a backstop for a host that has gone away, not
/// the mechanism, so the normal path costs a few milliseconds.
fn handover(serial: &mut Link, clock: &mut Millis) {
    let mut idle = 0;
    while idle < HANDOVER_TIMEOUT_MS {
        if serial.try_read_byte().is_some() {
            return;
        }
        if clock.tick() {
            idle += 1;
        }
    }
    rprintln!("boot: host did not acknowledge, handing over anyway");
}

/// Stream `len` bytes from the link into the inactive bank and swap.
/// Returns only on failure, having reported it to the host.
fn install(
    boot: Boot<Unverified>,
    store: &mut BkupRamStore<{ link::STORE_OFFSET }>,
    serial: &mut Link,
    clock: &mut Millis,
    len: u32,
) -> Boot<Unverified> {
    rprintln!("boot: installing {} bytes", len);
    let record = store.read().unwrap_or_default();
    serial.take_rx_error();

    let source = ImageStream {
        serial: &mut *serial,
        clock,
        left: len,
    };
    let Aborted { error, boot } = boot.install(store, record, source);

    if serial.take_rx_error() {
        rprintln!("boot: receive error during the image body (overrun?)");
    }
    let status = match error {
        InstallError::Flash(FlashError::ImageTooLarge) => Status::TooLarge,
        InstallError::Flash(FlashError::Nvm(_)) => Status::FlashError,
        InstallError::Verify(_) => Status::VerifyFailed,
        InstallError::Write(_) => Status::FlashError,
    };
    rprintln!("boot: install failed: {:?}", status);
    send(serial, &Message::UpdateResult(status));
    boot
}

/// Exactly `left` bytes off the wire, one blocking read per `next`, so the
/// image flows page by page into flash and is never buffered whole. A
/// stalled host truncates the stream rather than wedging the bootloader:
/// the short image fails to verify and is reported like any other bad one.
struct ImageStream<'a> {
    serial: &'a mut Link,
    clock: &'a mut Millis,
    left: u32,
}

impl Iterator for ImageStream<'_> {
    type Item = u8;

    fn next(&mut self) -> Option<u8> {
        self.left = self.left.checked_sub(1)?;
        let mut idle = 0;
        loop {
            if let Some(b) = self.serial.try_read_byte() {
                return Some(b);
            }
            if self.clock.tick() {
                idle += 1;
                if idle >= BYTE_TIMEOUT_MS {
                    self.left = 0;
                    return None;
                }
            }
        }
    }
}

fn send(serial: &mut Link, msg: &Message) {
    let mut buf = [0u8; proto::MAX_FRAME];
    // MAX_FRAME is sized for the largest Message, so this cannot fail.
    if let Ok(frame) = proto::encode(msg, &mut buf) {
        serial.write_all(frame);
        // Drops this node's own echo off the half-duplex bus, not host traffic.
        serial.flush_rx();
    }
}

/// Millisecond ticker on SysTick, polled rather than interrupt driven: the
/// bootloader runs with no vector table of its own beyond the reset entry.
struct Millis {
    syst: SYST,
}

impl Millis {
    fn new(mut syst: SYST) -> Self {
        syst.set_clock_source(SystClkSource::Core);
        syst.set_reload(link::CORE_CLOCK_HZ / 1000 - 1);
        syst.clear_current();
        syst.enable_counter();
        Self { syst }
    }

    /// True once per elapsed millisecond. COUNTFLAG clears on read, so a
    /// caller slower than 1 kHz loses ticks and its timeout stretches;
    /// both callers here only ever wait on the wire between polls.
    fn tick(&mut self) -> bool {
        self.syst.has_wrapped()
    }
}

fn park() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    park()
}
