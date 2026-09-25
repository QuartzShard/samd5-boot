//! An A/B firmware-update bootloader for Microchip SAM D5x/E5x
//! (`ATSAMD5x`/`ATSAME5x`) MCUs, built on the silicon's native dual-bank
//! swap (`BKSWRST`) rather than a software copy engine.
//!
//! No single failure leaves the device unbootable: a power cut mid-write,
//! an image that fails verification, and a trial boot that never confirms
//! each leave a bootable image mapped. An identical BOOT binary is
//! bench-installed at the head of both banks and is never rewritten by a
//! field update, so the revert path is present in whichever bank the chip
//! resets into.
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
//! stays the downstream project's file. Each binary still needs the usual
//! `cortex-m-rt` build script to copy its own `memory.x` into `OUT_DIR` and
//! put that on the link search path; see
//! `examples/update-rig/boot/build.rs`.
//!
//! Both binaries must build with the same part and `bootprot-*` features,
//! since the geometry is frozen per device once BOOT is installed.
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
//! It returns for download mode when the application asked for an update,
//! when nothing on the part is known to boot, or when a store write failed
//! and the promotion, trial or revert this boot called for could not be
//! recorded. In download mode the binary drives its own transport and calls
//! [`Boot::install`], which streams an image into the inactive slot, records
//! a trial, and swaps. When to give up (a silence timeout, a retry budget)
//! is the binary's policy, since it owns the transport. [`Boot::install`]
//! takes its image source as a closure rather than an iterator, because it
//! condemns the bank it is about to overwrite before it reads a byte: the
//! closure runs between that record write and the first erase, which is
//! where a receiver is declared ready. [`Boot::verify`]
//! gates the jump into the active image and [`Boot::revert`] condemns it and
//! swaps back.
//!
//! ## Application flow
//!
//! The application links this crate for [`client::BootClient`], built over
//! the same [`persist::BootStorage`] the bootloader uses (the app constructs
//! a store at the same offset and hands it in). Early in init it calls
//! [`confirm`] to mark a trial image good and take over the trial watchdog,
//! or [`reject`] to condemn the running image; [`request_update`] asks BOOT
//! to enter download mode on the next boot, and [`boot_state`] reads the
//! previous boot's outcome for upstream reporting.
//!
//! `examples/update-rig/` in the repository is the worked integration and
//! the test suite: a BOOT and an application driven from a host over RTT
//! (the default, needing only a debug probe) or RS485, covering install,
//! verify, trial, confirm, reject, rollback and the update window, and
//! reporting one PASS/FAIL line each. `cargo xtask test --chip <CHIP>`
//! builds it, stamps the images, places BOOT at both bank heads and runs
//! the suite.
//!
//! # Banks and slots
//!
//! Two terms are used precisely throughout:
//!
//! - A *bank* is a physical flash half (A or B, the `STATUS.AFIRST`
//!   domain). Persisted per-bank state ([`persist::BootState`]) is keyed by
//!   physical bank and resolved through `Nvm::first_bank`, so the record
//!   survives a swap while the mapping flips.
//! - A *slot* is a mapped position. The active slot is always at the base
//!   of flash ([`ACTIVE_SLOT_ADDR`]); the inactive slot is in the upper
//!   half ([`INACTIVE_SLOT_ADDR`]).
//!
//! `BKSWRST` (one NVMCTRL command) flips which bank occupies which slot,
//! reallocates any live SmartEEPROM reserve into the other bank, and resets.
//! The command is a single commit point: a power loss yields the old
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
//! # Host builds
//!
//! Nothing is enabled by default, so a plain dependency is the flash ABI
//! alone and compiles anywhere: [`consts`] (with [`consts::geometry`] for
//! parts chosen at runtime), [`manifest`], [`boot_info`], [`crc32`], and
//! the record types in [`persist`]. Selecting a part feature (for example
//! `samd51j20a`) brings the `target` feature, the hal, and everything that
//! touches the silicon. Host tooling depends on the crate with no features,
//! so the tooling and the firmware cannot disagree about a layout or a fuse
//! encoding.
//!
//! [`ACTIVE_SLOT_ADDR`]: consts::ACTIVE_SLOT_ADDR
//! [`INACTIVE_SLOT_ADDR`]: consts::INACTIVE_SLOT_ADDR
//! [`confirm`]: client::BootClient::confirm
//! [`reject`]: client::BootClient::reject
//! [`request_update`]: client::BootClient::request_update
//! [`boot_state`]: client::BootClient::boot_state
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
