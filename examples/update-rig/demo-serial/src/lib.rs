//! Blocking half-duplex RS485 serial over SERCOM5 at 115200 8N1, for the
//! samd5-boot download-mode stress rig on the ATSAMD51J20A.
//!
//! Three pins, and this crate touches no others: PB02 = TxD, PB03 = RxD (both
//! peripheral function D = SERCOM5 PAD[0] and PAD[1]), PB00 = the RS485 driver
//! enable, held as a plain GPIO output.
//!
//! # Clock: GCLK generator 0, DFLL48M open loop, 48 MHz
//!
//! [`Serial::new`] does the two things the SERCOM needs and nothing more:
//! unmask `CLK_SERCOM5_APB` (SERCOM5 sits on the APBD bridge, and
//! `MCLK.APBDMASK` resets to 0, so this is required) and point the SERCOM5
//! core peripheral channel at generator 0.
//!
//! Generator 0 is `GCLK_MAIN` and needs no bring-up: DS 7.3.2 "Starting of
//! Clocks" says that out of reset the device runs on DFLL48M in open-loop
//! mode at 48 MHz, and that DFLL48M is generator 0's source. Every other
//! generic clock is off. Choosing generator 0 therefore costs no clock-tree
//! configuration and cannot collide with samd5-boot, which owns NVMCTRL and
//! would fight `GenericClockController` for it.
//!
//! The cost is accuracy. DS Table 54-50 gives open-loop DFLL48M with the reset
//! `DFLLVAL` as 45.8 to 49.3 MHz over [-40, 85] degC and 47.2 to 48.81 MHz
//! over [0, 60] degC, typical 47.972 MHz. Against DS Table 34-3 the receiver
//! accepts roughly +3.3%/-4.35% on a 10-bit frame at 16x, with +/-1.5%
//! recommended for margin, so a bench link at room temperature is comfortable
//! and the full industrial range is not. Tightening it means closed-loop DFLL
//! off XOSC32K, which needs PA00/PA01; out of scope here.
//!
//! **If downstream firmware reconfigures generator 0 (for example DPLL0 at 120
//! MHz), [`BAUD_REG`] is wrong and the link will not work.** Leave GCLK0 at
//! its reset configuration, or recompute the constant.
//!
//! # Transmit enable: plain GPIO on PB00, released on TXC
//!
//! PB00's function D really is SERCOM5 PAD[2], the RTS/TE pad, so the SERCOM
//! could drive it itself (DS 34.6.3.6: `CTRLA.FORM` = 0x0 with
//! `CTRLA.TXPO` = 0x3 puts the part in RS485 mode, holds TE through the stop
//! bits, and adds `CTRLC.GTIME` guard time). This crate drives PB00 by hand
//! instead, which leaves the SERCOM on `TXPO` = 0x0 and the link usable as a
//! plain UART. Switching to hardware TE is a three-line change: `txpo_3()`
//! instead of `txpo_0()`, PB00 into `AlternateD`, and drop the TE handling
//! from [`Serial::write_all`].
//!
//! The part that has to be right either way is the release edge.
//! [`Serial::write_all`] waits on `INTFLAG.TXC`, which sets only once the
//! shift register has emptied and no byte is queued in DATA, never on
//! `INTFLAG.DRE`, which sets a whole frame earlier. It also clears a stale TXC
//! before the burst so the flag it observes is its own last byte.
//!
//! # Why the PAC and not the HAL's UART
//!
//! `atsamd-hal`'s pad filter rejects RX on PAD[1] together with `TXPO` = 0x0
//! (`sercom/uart/pads_thumbv7em.rs`, "RX can't be Pad1 if TXPO is 0 because of
//! XCK conflict"), which is exactly the wiring here. The filter is
//! conservative: `TXPO` only assigns XCK "when applicable" (DS Table 34-2),
//! and DS 34.6.2.3 confirms XCK is routed to a pad only in synchronous mode,
//! which `CTRLA.CMODE` = 0 rules out. The SERCOM is therefore configured
//! through the PAC; pin muxing still goes through the gpio HAL.

#![no_std]

use atsamd_hal::ehal::digital::OutputPin;
use atsamd_hal::gpio::{AlternateD, D, PB00, PB02, PB03, Pin, PushPullOutput, Reset};
use atsamd_hal::pac;

pub use demo_rig::{CORE_CLOCK_HZ, STORE_OFFSET};

/// Names this transport in log output, so which one a build is using is
/// visible without reading the features it was compiled with.
pub const NAME: &str = "RS485";

/// Line rate.
pub const BAUD_RATE: u32 = 115_200;

/// Samples per bit, matching `CTRLA.SAMPR` = 16x arithmetic.
const OVERSAMPLE: u32 = 16;

/// Generic clock peripheral channel index for `GCLK_SERCOM5_CORE`
/// (DS Table 14-9 "PCHCTRLm Mapping").
const GCLK_SERCOM5_CORE: usize = 35;

/// `BAUD` for asynchronous arithmetic mode, `65536 * (1 - S * f_baud / f_ref)`
/// (DS Table 33-2), rounded to nearest.
///
/// At 48 MHz and 115200 this is 63019, an actual 115219.1 baud (+0.017%). The
/// intermediate product needs 64 bits.
pub const BAUD_REG: u16 = {
    let fref = CORE_CLOCK_HZ as u64;
    let scaled = 65536 * OVERSAMPLE as u64 * BAUD_RATE as u64;
    (65536 - (scaled + fref / 2) / fref) as u16
};

// DS Table 33-2 states the arithmetic-mode equation holds for f_baud <= f_ref/S.
const _: () = assert!(BAUD_RATE * OVERSAMPLE <= CORE_CLOCK_HZ);

use atsamd_hal::pac::interrupt;
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

/// Bytes the receive ring holds.
///
/// The consumer stalls for milliseconds at a time while NVMCTRL programs a
/// page or erases a block, and the SERCOM's own FIFO is two deep. Filling
/// this ring from the RXC interrupt is what lets an image stream survive
/// those stalls: at 115200 it covers about 700 ms of line rate, against a
/// worst case block erase of tens of milliseconds.
pub const RX_RING: usize = 8192;

struct Ring(UnsafeCell<[u8; RX_RING]>);
// SAFETY: one producer (the SERCOM5 RXC interrupt) and one consumer (whoever
// holds `Serial`), and they never touch the same slot: the producer writes at
// HEAD only after checking it is not about to meet TAIL, and the consumer
// reads at TAIL only while it trails HEAD.
unsafe impl Sync for Ring {}

static RX_BUF: Ring = Ring(UnsafeCell::new([0; RX_RING]));
static RX_HEAD: AtomicUsize = AtomicUsize::new(0);
static RX_TAIL: AtomicUsize = AtomicUsize::new(0);
static RX_LOST: AtomicBool = AtomicBool::new(false);

/// SERCOM5 interrupt line 2 is RXC on a USART.
#[interrupt]
fn SERCOM5_2() {
    // SAFETY: this handler touches only the receive side, and `Serial` masks
    // the line while it drives the bus, so it cannot race the transmit path.
    let sercom = unsafe { pac::Sercom5::steal() };
    let usart = sercom.usart_int();

    let status = usart.status().read();
    if status.bufovf().bit_is_set() || status.ferr().bit_is_set() || status.perr().bit_is_set() {
        RX_LOST.store(true, Ordering::Relaxed);
        usart.status().write(|w| {
            w.bufovf().set_bit();
            w.ferr().set_bit();
            w.perr().set_bit()
        });
        usart.intflag().write(|w| w.error().set_bit());
    }

    // Drain the FIFO: reading DATA is what clears RXC.
    while usart.intflag().read().rxc().bit_is_set() {
        let byte = usart.data().read().data().bits() as u8;
        let head = RX_HEAD.load(Ordering::Relaxed);
        let next = (head + 1) % RX_RING;
        if next == RX_TAIL.load(Ordering::Acquire) {
            RX_LOST.store(true, Ordering::Relaxed);
        } else {
            unsafe { (*RX_BUF.0.get())[head] = byte };
            RX_HEAD.store(next, Ordering::Release);
        }
    }
}

/// The transport surface `boot` and `app` are written against, which
/// `demo-rtt` also provides. Only the constructor differs.
pub use Serial as Link;

/// Blocking half-duplex RS485 UART on SERCOM5.
///
/// Owns the SERCOM and all three pins, so nothing else can repurpose the link
/// while it is live. Transmit and receive are strictly alternating: the driver
/// is enabled only for the duration of a [`write_all`](Self::write_all).
pub struct Serial {
    sercom: pac::Sercom5,
    te: Pin<PB00, PushPullOutput>,
    _tx: Pin<PB02, AlternateD>,
    _rx: Pin<PB03, AlternateD>,
}

impl Serial {
    /// Bring up the link.
    ///
    /// `mclk` and `gclk` are borrowed only to unmask the APB clock and route
    /// the core clock; see the module docs for what is assumed about
    /// generator 0.
    pub fn new(
        sercom: pac::Sercom5,
        mclk: &mut pac::Mclk,
        gclk: &mut pac::Gclk,
        tx: Pin<PB02, Reset>,
        rx: Pin<PB03, Reset>,
        te: Pin<PB00, Reset>,
    ) -> Self {
        mclk.apbdmask().modify(|_, w| w.sercom5_().set_bit());

        // DS 14.6.3.3 wants the channel off while its generator changes. It is
        // already off coming out of reset, so the clear is only insurance
        // against an earlier boot stage; deliberately not polled, because a
        // channel left enabled on a stopped generator would never report the
        // disable and would hang here. The enable is polled: CHEN reading back
        // '1' is what proves the core clock is live before the SERCOM's own
        // SYNCBUSY loops start waiting on it.
        let pchctrl = gclk.pchctrl(GCLK_SERCOM5_CORE);
        pchctrl.modify(|_, w| w.chen().clear_bit());
        pchctrl.modify(|_, w| w.r#gen().gclk0());
        pchctrl.modify(|_, w| w.chen().set_bit());
        while pchctrl.read().chen().bit_is_clear() {}

        let mut te = te.into_push_pull_output();
        // Idle low: driver tri-stated, this node listening. Errors are
        // `Infallible`.
        let _ = te.set_low();

        let mut serial = Self {
            sercom,
            te,
            _tx: tx.into_alternate::<D>(),
            _rx: rx.into_alternate::<D>(),
        };
        serial.configure();
        serial
    }

    fn configure(&mut self) {
        let usart = self.sercom.usart_int();

        usart.ctrla().write(|w| w.swrst().set_bit());
        while usart.syncbusy().read().swrst().bit_is_set() {}

        usart.ctrla().modify(|_, w| {
            w.mode().usart_int_clk();
            w.cmode().async_();
            w.form().usart_frame_no_parity();
            w.sampr()._16x_arithmetic();
            w.rxpo().pad1(); // PB03
            w.txpo().txpo_0(); // PB02
            w.dord().lsb()
        });

        // Not enable-protected and not synchronized; write before ENABLE.
        // SAFETY: BAUD is a plain 16-bit divisor with no reserved encodings.
        usart.baud().write(|w| unsafe { w.baud().bits(BAUD_REG) });

        usart.ctrlb().modify(|_, w| {
            w.chsize()._8_bit();
            w.sbmode()._1_bit()
        });
        while usart.syncbusy().read().ctrlb().bit_is_set() {}

        // RXEN and TXEN each raise SYNCBUSY.CTRLB on their own.
        usart.ctrlb().modify(|_, w| w.rxen().set_bit());
        while usart.syncbusy().read().ctrlb().bit_is_set() {}
        usart.ctrlb().modify(|_, w| w.txen().set_bit());
        while usart.syncbusy().read().ctrlb().bit_is_set() {}

        usart.ctrla().modify(|_, w| w.enable().set_bit());
        while usart.syncbusy().read().enable().bit_is_set() {}

        usart.intenset().write(|w| w.rxc().set_bit());
        // SAFETY: the handler above owns the receive side from here on.
        unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::SERCOM5_2) };
    }

    /// Drive the bus for exactly as long as it takes to send `bytes`.
    ///
    /// Returns once the last stop bit is on the wire and the driver is off, so
    /// the caller may listen for a reply immediately. An empty slice is a
    /// no-op: asserting the driver would leave TXC waiting on a transmission
    /// that never starts.
    pub fn write_all(&mut self, bytes: &[u8]) {
        if bytes.is_empty() {
            return;
        }
        // Nothing can legitimately arrive while this node drives the pair, and
        // masking keeps the handler off DATA for the duration.
        cortex_m::peripheral::NVIC::mask(pac::Interrupt::SERCOM5_2);

        let _ = self.te.set_high();
        {
            let usart = self.sercom.usart_int();
            // A previous burst leaves TXC set; clear it so the flag waited on
            // below belongs to this burst's final byte.
            usart.intflag().write(|w| w.txc().set_bit());

            for &b in bytes {
                while usart.intflag().read().dre().bit_is_clear() {}
                // SAFETY: DATA is a full-width field; an 8-bit value is in range.
                unsafe { usart.data().write(|w| w.data().bits(b as u32)) };
            }

            while usart.intflag().read().txc().bit_is_clear() {}
        }
        let _ = self.te.set_low();
        // SAFETY: as in `new`.
        unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::SERCOM5_2) };
    }

    /// Block until a byte arrives.
    ///
    /// Never returns if the peer goes quiet; use [`try_read_byte`](Self::try_read_byte)
    /// where a timeout is needed.
    pub fn read_byte(&mut self) -> u8 {
        loop {
            if let Some(b) = self.try_read_byte() {
                return b;
            }
        }
    }

    /// Take a received byte if one is already waiting.
    ///
    /// Poll this against the caller's own deadline to get a timeout window.
    /// Receive errors do not surface here: they are latched for
    /// [`take_rx_error`](Self::take_rx_error) and the byte, valid or not, is
    /// still returned.
    pub fn try_read_byte(&mut self) -> Option<u8> {
        let tail = RX_TAIL.load(Ordering::Relaxed);
        if tail == RX_HEAD.load(Ordering::Acquire) {
            return None;
        }
        let byte = unsafe { (*RX_BUF.0.get())[tail] };
        RX_TAIL.store((tail + 1) % RX_RING, Ordering::Release);
        Some(byte)
    }

    /// Report and reset whether any framing, parity, or overflow error has been
    /// seen since the last call.
    ///
    /// Overflow is the one to watch on this rig: the receive FIFO is two deep,
    /// and a caller that stalls for more than ~170 us at 115200 (an NVMCTRL
    /// page write, say) drops bytes out of the middle of an image stream.
    pub fn take_rx_error(&mut self) -> bool {
        RX_LOST.swap(false, Ordering::Relaxed)
    }

    /// Silence the link before handing the core to another image.
    ///
    /// The receive interrupt would otherwise still be live after the jump,
    /// vectoring through whatever table the new image installed.
    pub fn quiesce(&mut self) {
        cortex_m::peripheral::NVIC::mask(pac::Interrupt::SERCOM5_2);
        self.sercom
            .usart_int()
            .intenclr()
            .write(|w| w.rxc().set_bit());
    }

    /// Re-arm receive after a [`quiesce`](Self::quiesce) that did not end in
    /// a handoff.
    pub fn arm(&mut self) {
        self.sercom
            .usart_int()
            .intenset()
            .write(|w| w.rxc().set_bit());
        // SAFETY: as in `new`.
        unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::SERCOM5_2) };
    }

    /// Discard everything buffered on the receive side.
    ///
    /// Worth calling after [`write_all`](Self::write_all): a board that leaves
    /// the transceiver's receiver enabled hears this node's own burst, and one
    /// that ties receiver-enable to PB00 instead floats the receiver output
    /// while this node drives, which can fake a start bit.
    pub fn flush_rx(&mut self) {
        while self.try_read_byte().is_some() {}
    }
}

