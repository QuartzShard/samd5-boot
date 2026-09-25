//! The application image for the update rig: the payload BOOT installs,
//! swaps into, and boots. APP role, so its `memory.x` selects
//! `samd5_boot_app.x` and it reserves a manifest slot with
//! [`install_manifest!`]; the manifest tool stamps the length and CRCs into
//! that slot after linking, which is what makes the image verify.
//!
//! On the wire it answers `Ping` with `Pong`, and `GetState` with what it is
//! running, which is the host's proof that a boot actually completed. `Update`
//! is an order to flag an update request and reset; BOOT owns the update
//! itself, waiting for the host's `BeginUpdate` once the request flag is set,
//! so nothing here touches flash.
//!
//! BOOT keeps its trial record in backup RAM by default, so a freshly
//! installed image boots on trial: this image calls
//! [`confirm`](samd5_boot::client::BootClient::confirm) early in `main` to
//! mark itself good, which also takes the trial watchdog off BOOT's terms.
//! Building with `--features noconfirm` skips that call, which is how the
//! rig exercises the auto-revert path. `--features see-store` moves the
//! record to SmartEEPROM instead, which BOOT must be built with too.
//!
//! Under `--features rs485` the link's baud divisor assumes GCLK generator 0
//! is still the reset DFLL48M at 48 MHz; deliberately nothing here
//! reconfigures the clock tree.

#![no_std]
#![no_main]

use atsamd_hal as hal;
use hal::watchdog::{Watchdog, WatchdogTimeout};
use proto::{Decoder, Feed, MAX_FRAME, Message, encode};
use rtt_target::rprintln;
use samd5_boot::{
    client::BootClient,
    install_manifest,
    manifest::AppManifest,
    persist::{BankState, RevertReason},
};

#[cfg(not(feature = "see-store"))]
use samd5_boot::persist::BkupRamStore;
#[cfg(feature = "see-store")]
use samd5_boot::persist::SmartEepromStore;

/// Where the boot record lives. BOOT must be built to match, or the two do
/// not see each other's writes.
#[cfg(not(feature = "see-store"))]
type Store = BkupRamStore<{ link::STORE_OFFSET }>;
#[cfg(feature = "see-store")]
type Store = SmartEepromStore<0>;

/// Identifies this build on the wire, so the host can tell which image a
/// device came back running. The no-confirm build is a different image.
#[cfg(not(feature = "noconfirm"))]
pub const APP_VERSION: u16 = 1;
#[cfg(feature = "noconfirm")]
pub const APP_VERSION: u16 = 2;

// The transport is whichever crate the `rtt` / `rs485` feature selected.
// Both expose the same `Link` surface; only the constructor differs, which
// is the one thing cfg'd at the call site below.
#[cfg(feature = "rs485")]
use demo_serial as link;
#[cfg(feature = "rtt")]
use demo_rtt as link;
use link::Link;

install_manifest!(AppManifest::placeholder());

#[cortex_m_rt::entry]
fn main() -> ! {
    // Only the RS485 constructor borrows these mutably.
    #[cfg_attr(feature = "rtt", allow(unused_mut))]
    let mut peripherals = hal::pac::Peripherals::take().expect("Infallible");
    let mut wdt = Watchdog::new(peripherals.wdt);

    // Bringing the link up first also sets up `rprintln!`: over RTT the two
    // share one control block, so the transport owns printing.
    #[cfg(feature = "rtt")]
    let mut serial = Link::new();
    #[cfg(feature = "rs485")]
    let mut serial = {
        rtt_target::rtt_init_print!();
        let pins = hal::gpio::Pins::new(peripherals.port);
        Link::new(
            peripherals.sercom5,
            &mut peripherals.mclk,
            &mut peripherals.gclk,
            pins.pb02,
            pins.pb03,
            pins.pb00,
        )
    };
    rprintln!("app: booted from the active slot");
    // Whatever the bus did while this node was resetting is not a frame.
    serial.flush_rx();

    let mut decoder = Decoder::new();

    let Some(store) = open_store() else {
        rprintln!("app: SmartEEPROM unusable, run `cargo xtask provision --sblk 1`");
        park()
    };
    let mut client = BootClient::new(store);
    // Read only, to name which bank this image is running from.
    let nvm = hal::nvm::Nvm::new(peripherals.nvmctrl);
    // An unreadable store and a code this build cannot decode both report
    // here as no revert.
    let reason = client
        .boot_state()
        .map_or(RevertReason::None, |s| s.revert_reason.unwrap_or_default());
    let revert_reason = reason as u8;
    if reason != RevertReason::None {
        rprintln!("app: previous image was rolled back, reason {}", revert_reason);
    }

    // Marking this boot good is what promotes the image out of trial on the
    // next boot, and hands the watchdog over on this one.
    let confirmed = if cfg!(feature = "noconfirm") {
        rprintln!("app: NOT confirming (auto-revert build)");
        false
    } else {
        match client.confirm(&mut wdt, Some(WatchdogTimeout::Cycles16K)) {
            Ok(_) => {
                rprintln!("app: confirmed");
                true
            }
            Err(_) => {
                rprintln!("app: confirm failed");
                false
            }
        }
    };

    rprintln!("app: listening on the {} link", link::NAME);
    loop {
        let byte = serial.read_byte();
        // Byte at a time: this side never receives a raw image, so a
        // completed frame never leaves a tail that has to be fed back.
        let Feed::Frame { msg, .. } = decoder.feed(&[byte]) else {
            continue;
        };

        match msg {
            Message::Ping(payload) => {
                send(&mut serial, &Message::Pong(payload));
                rprintln!("app: pong");
            }
            Message::Update => {
                rprintln!("app: requesting an update, resetting into BOOT");
                // The flag is what makes BOOT wait for the host rather than
                // booting on: no window to race.
                let _ = client.request_update();
                cortex_m::peripheral::SCB::sys_reset();
            }
            Message::Reset => {
                rprintln!("app: resetting");
                cortex_m::peripheral::SCB::sys_reset();
            }
            Message::GetState => {
                let state = Message::State {
                    app_version: APP_VERSION,
                    revert_reason,
                    confirmed,
                };
                send(&mut serial, &state);
            }
            Message::GetBanks => {
                // An unreadable record reports as two `None`s, which is what
                // a fresh one holds anyway.
                let banks = client.banks(&nvm).ok();
                send(
                    &mut serial,
                    &Message::Banks {
                        active: banks.as_ref().map_or(BankState::None, |b| b.active) as u8,
                        inactive: banks.as_ref().map_or(BankState::None, |b| b.inactive) as u8,
                    },
                );
            }
            Message::Reject => {
                rprintln!("app: condemning this image, resetting to roll back");
                let _ = client.reject();
                cortex_m::peripheral::SCB::sys_reset();
            }
            // Pong / BeginUpdate / UpdateResult / Banks are not this side's
            // traffic.
            _ => {}
        }
    }
}

#[cfg(not(feature = "see-store"))]
fn open_store() -> Option<Store> {
    // SAFETY: the same backup-RAM record BOOT keeps, at the same offset.
    Some(unsafe { BkupRamStore::new() })
}

#[cfg(feature = "see-store")]
fn open_store() -> Option<Store> {
    SmartEepromStore::new().ok()
}

fn send(serial: &mut Link, msg: &Message) {
    let mut buf = [0u8; MAX_FRAME];
    // MAX_FRAME is sized for the largest Message, so this cannot fail.
    if let Ok(frame) = encode(msg, &mut buf) {
        serial.write_all(frame);
        // Drops this node's own echo off the half-duplex bus, not host traffic.
        serial.flush_rx();
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
