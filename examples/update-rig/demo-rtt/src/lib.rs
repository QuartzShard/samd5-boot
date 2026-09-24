//! A byte stream over RTT, for a rig that has nothing but a debug probe.
//!
//! RTT is two ring buffers in the target's RAM and a host that reads and
//! writes them over the debug port while the core runs. Every board that can
//! be flashed already has what this needs; `demo-serial` wants a transceiver
//! on three specific pins in exchange for proving an update with no debugger
//! in the loop.
//!
//! # Why the bootloader stops needing flow control
//!
//! The hard part of streaming an image into `Boot::install` is that the
//! writer stops reading for tens of milliseconds at every page program, and
//! at every block erase. Over a UART those bytes are simply gone, which is
//! why `demo-serial` carries an 8 KiB interrupt-fed ring.
//!
//! Here the buffer *is* the transport. The host writes into a ring in RAM
//! and the target drains it whenever it gets around to it; if the ring fills,
//! the host's write reports a short count and it tries again. Nothing is
//! lost because nothing was ever in flight.
//!
//! # Channels
//!
//! Channel 0 up is the log channel, named `Terminal` and set as the
//! `rprintln!` sink, exactly what `rtt_init_print!` would have built. The
//! protocol gets channel 1 up and channel 0 down, both named `samd5-boot`
//! so the host can confirm it attached to the block it meant to.

#![no_std]

use rtt_target::{ChannelMode, DownChannel, UpChannel, rtt_init};

pub use demo_rig::{CORE_CLOCK_HZ, STORE_OFFSET};

/// Names this transport in log output, so which one a build is using is
/// visible without reading the features it was compiled with.
pub const NAME: &str = "RTT";

/// Host to target: a control frame, or the raw body of an image. Eight flash
/// pages deep, so the host can keep writing while the target is busy in a
/// page program or a 16-page block erase.
pub const RX_RING: usize = 4096;
/// Target to host: only ever control frames, which are tens of bytes.
pub const TX_RING: usize = 1024;
/// Log output. Shares the block with the protocol channels.
pub const LOG_RING: usize = 1024;

pub struct Link {
    tx: UpChannel,
    rx: DownChannel,
    /// Batches down-channel reads, so `try_read_byte` costs one read per 64
    /// bytes rather than one per byte.
    buf: [u8; 64],
    head: usize,
    tail: usize,
}

impl Link {
    /// Bring up the RTT control block and claim the protocol channels.
    ///
    /// Call once. Channel 0 up becomes the `rprintln!` sink, exactly as
    /// `rtt_init_print!` would have set it up.
    pub fn new() -> Self {
        let channels = rtt_init! {
            up: {
                0: { size: LOG_RING, mode: ChannelMode::NoBlockSkip, name: "Terminal" }
                // A dropped reply is a failed exchange, and the host is
                // always reading, so waiting is better than skipping.
                1: { size: TX_RING, mode: ChannelMode::BlockIfFull, name: "samd5-boot" }
            }
            down: {
                0: { size: RX_RING, mode: ChannelMode::BlockIfFull, name: "samd5-boot" }
            }
        };
        rtt_target::set_print_channel(channels.up.0);
        publish_control_block();

        Self {
            tx: channels.up.1,
            rx: channels.down.0,
            buf: [0; 64],
            head: 0,
            tail: 0,
        }
    }

    pub fn write_all(&mut self, bytes: &[u8]) {
        let mut rest = bytes;
        while !rest.is_empty() {
            // BlockIfFull, so a short write means the host has not drained
            // the ring yet rather than that anything was lost.
            let n = self.tx.write(rest);
            rest = &rest[n..];
        }
    }

    pub fn try_read_byte(&mut self) -> Option<u8> {
        if self.head == self.tail {
            self.head = 0;
            self.tail = self.rx.read(&mut self.buf);
            if self.tail == 0 {
                return None;
            }
        }
        let b = self.buf[self.head];
        self.head += 1;
        Some(b)
    }

    pub fn read_byte(&mut self) -> u8 {
        loop {
            if let Some(b) = self.try_read_byte() {
                return b;
            }
        }
    }

    /// Drop anything already received.
    pub fn flush_rx(&mut self) {
        self.head = 0;
        self.tail = 0;
        while self.rx.read(&mut self.buf) != 0 {}
    }

    /// Always false: a full ring stalls the host rather than overrunning, so
    /// there is no receive error to report. Present so the two transports
    /// offer the same surface.
    pub fn take_rx_error(&mut self) -> bool {
        false
    }

    /// No-ops, kept for parity with `demo-serial`, whose receive path is
    /// interrupt driven and must not outlive the image that installed its
    /// vector table. Nothing here runs from an interrupt.
    pub fn quiesce(&mut self) {}
    pub fn arm(&mut self) {}
}

/// Tell the host where the control block ended up.
///
/// `rtt_init!` places it wherever the linker chose, which differs between
/// BOOT and the application, so the host has to be told rather than left to
/// search. See [`demo_rig::RTT_POINTER_OFFSET`] for why the slot lives in
/// backup RAM.
fn publish_control_block() {
    // SAFETY: `rtt_init!` above exported this symbol, and the slot is ours
    // by the rig's backup-RAM layout.
    unsafe {
        let block = &raw const _SEGGER_RTT as u32;
        let slot = demo_rig::RTT_POINTER_ADDR as *mut u32;
        // Address first, magic second: a torn write is then not trusted.
        slot.add(1).write_volatile(block);
        slot.write_volatile(demo_rig::RTT_POINTER_MAGIC);
    }
}

unsafe extern "C" {
    #[link_name = "_SEGGER_RTT"]
    static _SEGGER_RTT: u8;
}
