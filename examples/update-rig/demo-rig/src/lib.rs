//! Facts about the rig that more than one crate has to agree on, and that
//! belong to the board rather than to any one transport or firmware role.
//!
//! Each of these is a place where two crates holding their own copy would
//! be wrong in a way nothing catches until the rig misbehaves, so they live
//! here and are imported rather than restated.

#![no_std]

use samd5_boot::{consts::BKUPRAM_ADDR, persist::BootStore};

/// Where the boot record sits in backup RAM.
///
/// BOOT writes it and the application reads it, so the two must agree
/// exactly; a mismatch is silent until a confirm fails to take and every
/// image reverts.
pub const STORE_OFFSET: usize = 0;

/// GCLK generator 0 out of reset: DFLL48M in open loop (DS 7.3.2). The demo
/// deliberately does not touch the clock tree, so this is what the core runs
/// at, and what the SERCOM5 core channel is fed from in the RS485 build.
pub const CORE_CLOCK_HZ: u32 = 48_000_000;

/// Where the firmware publishes the address of its RTT control block.
///
/// The host would otherwise have to search RAM for it, which is far too slow
/// to repeat: a trial image is watchdog-reset every few seconds by design,
/// and a scan of the whole of RAM does not reliably fit between two resets.
/// The block itself cannot simply be pinned, because a `static` is placed by
/// the linker and reserving a RAM region is a linker job. Backup RAM is the
/// way out: the linker never allocates from it, so a literal offset here
/// means the same thing in every image and on the host.
///
/// Layout is [`RTT_POINTER_MAGIC`] then the address, both little-endian.
/// The magic is written last, so a half-written slot is not trusted.
pub const RTT_POINTER_OFFSET: usize = STORE_OFFSET + size_of::<BootStore>();
pub const RTT_POINTER_ADDR: usize = BKUPRAM_ADDR + RTT_POINTER_OFFSET;
/// ASCII `rttp`.
pub const RTT_POINTER_MAGIC: u32 = 0x7274_7470;
