//! An A/B firmware-update bootloader for Microchip SAM D5x/E5x
//! (`ATSAMD5x`/`ATSAME5x`) MCUs, built on the silicon's native dual-bank
//! swap (`BKSWRST`) rather than a software copy engine.
//!
//! The governing guarantee is that no single failure bricks the device: a
//! power cut at any byte, an accepted-but-broken image, or a wedged trial
//! boot always leaves a bootable image mapped. An identical BOOT binary is
//! bench-installed at the head of both banks and is never rewritten by a
//! field update, so the revert path is present in whichever bank the chip
//! resets into.
//!
//! # Banks and slots
//!
//! The vocabulary is load-bearing:
//!
//! - A *bank* is a physical flash half (A or B, the `STATUS.AFIRST`
//!   domain). Persisted per-bank state ([`persist::BootState`]) is keyed by
//!   physical bank and resolved through `Nvm::first_bank`, so the record
//!   survives a swap while the mapping flips.
//! - A *slot* is a mapped position. The active slot is always at the base
//!   of flash ([`ACTIVE_SLOT_ADDR`](consts::ACTIVE_SLOT_ADDR)); the inactive
//!   slot is in the upper half ([`INACTIVE_SLOT_ADDR`](consts::INACTIVE_SLOT_ADDR)).
//!
//! `BKSWRST` (one NVMCTRL command) flips which bank occupies which slot,
//! reallocates any live SmartEEPROM reserve into the other bank, and resets.
//! The fuse program is a single commit point: a power loss yields the old
//! mapping or the new one, never a half-swap. Nothing but software ever
//! flips the mapping back, so a revert is itself a swap.
//!
//! # Memory layout
//!
//! Within a slot (shown here for the active slot at `0x0`), the BOOT region
//! sits at the base under BOOTPROT and the application image begins just
//! above it. Two structures sit at frozen offsets so the BOOT and app
//! binaries agree on where to find them across independent builds:
//!
//! ```text
//! 0x0000_0000                    ┌──────────────────────────┐ ─┐
//!                                │ BOOT vector table + code │  │ BOOT region
//! BOOT_INFO_ADDR (top page)      │ boot-info block          │  │ (BOOTPROT)
//! BOOT_SIZE                      ├──────────────────────────┤ ─┤
//!                                │ app vector table         │  │
//! + MANIFEST_OFFSET (0x400)      │ app manifest             │  │ app image
//!                                │ app code                 │  │
//! bank top − SmartEEPROM reserve └──────────────────────────┘ ─┘
//! ```
//!
//! The boot-info block ([`boot_info`]) is embedded by BOOT and read by the
//! application to audit compatibility; the manifest ([`manifest`]) is
//! embedded by the application and read by BOOT to verify an image. Both are
//! append-only flash ABIs.
//!
//! # The two binaries
//!
//! A deployment is one library and two thin downstream binaries. Each owns
//! its own `memory.x`, which selects a role with a single line:
//! `INCLUDE samd5_boot_boot.x` for the BOOT binary or
//! `INCLUDE samd5_boot_app.x` for the application. `build.rs` generates
//! those fragments (MEMORY, the manifest/boot-info section placement, and
//! `_stext`) into `OUT_DIR` and exposes them via `rustc-link-search`, so
//! they compose with the stock `cortex-m-rt` `-Tlink.x` flow while `memory.x`
//! stays the downstream project's file. Both binaries must build with the
//! same part and `bootprot-*` features, since the geometry is frozen per
//! device once BOOT is installed.
//!
//! ## BOOT flow
//!
//! The BOOT binary constructs a [`Boot`] from the NVM, DSU, and watchdog
//! peripherals with [`Boot::new`], which validates the fuse configuration
//! rather than changing it: provisioning is a bench step, not something an
//! image does to itself. [`Boot::boot_or_enter_download`]
//! then classifies the persisted [`BootStore`](persist::BootStore) and acts
//! on it, booting the active image, swapping to roll back, or returning so
//! the binary can enter download mode; a caller wanting its own policy reads
//! [`Boot::disposition`] and matches the outcomes itself.
//!
//! It returns for download mode in exactly two cases: no bootable image in
//! either bank, or the application asked for an update. In download mode the
//! binary drives its own transport and calls [`Boot::install`], which streams
//! an image into the inactive slot, records a trial, and swaps. When to give
//! up (a silence timeout, a retry budget) is the binary's policy, since it
//! owns the transport. [`Boot::verify`] gates the boot-through-active path and
//! [`Boot::revert`] the rollback path.
//!
//! ## Application flow
//!
//! The application links this crate for [`client::BootClient`], built over
//! the same [`persist::BootStorage`] the bootloader uses (the app constructs
//! a store at the same offset and hands it in). Early in init it calls
//! [`confirm`](client::BootClient::confirm) to mark a trial image good and
//! take over the trial watchdog, or [`reject`](client::BootClient::reject)
//! to condemn the running image; [`request_update`](client::BootClient::request_update)
//! asks BOOT to enter download mode on the next boot, and
//! [`boot_state`](client::BootClient::boot_state) reads the previous boot's
//! outcome for upstream reporting.
//!
//! `examples/update-rig/` is the worked integration and the test suite: a
//! BOOT and an application talking to a host over RS485, driving every path
//! here and reporting one PASS/FAIL line each. `cargo xtask` builds, stamps,
//! provisions and flashes it.
//!
//! # Host builds
//!
//! Default features drive the silicon. With `--no-default-features` the
//! crate compiles anywhere and exposes the flash ABI alone: [`consts`] (with
//! [`consts::geometry`] for parts chosen at runtime), [`manifest`],
//! [`boot_info`], [`crc32`], and the record types in [`persist`]. That is
//! what `xtask` builds against, so the tooling and the firmware cannot
//! disagree about a layout or a fuse encoding.
#![no_std]

pub mod boot_info;
pub mod consts;
pub mod crc32;
pub mod manifest;
pub mod persist;

#[cfg(feature = "target")]
pub mod client;
#[cfg(feature = "target")]
mod flash_writer;

#[cfg(feature = "target")]
pub mod boot;

#[cfg(feature = "target")]
pub use boot::*;
#[cfg(feature = "target")]
pub use flash_writer::FlashError;
