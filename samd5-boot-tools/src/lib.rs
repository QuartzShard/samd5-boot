//! Bench and manufacturing operations for a samd5-boot device
//!
//! What to call, in the order a fresh part needs it:
//!
//! 1. [`provision::info`] reports what a part is configured as.
//! 1. [`provision::run`] writes the BOOTPROT and region-lock fuses and
//!    checks they survive a reset.
//! 1. [`flash::run`] places BOOT at the head of both banks, going round the
//!    protected head by the bank swap.
//! 1. [`image::stamp`] stamps an application image so BOOT accepts it. This
//!    one needs no probe.
//!
//! [`provision::request_update`] additionally sets the update request in the
//! boot record, so BOOT waits for an image instead of booting the
//! application.
//!
//! The layouts and encodings all come from [`samd5_boot`] itself, so a
//! device programmed by this agrees with the firmware by construction rather
//! than by two implementations matching.
//!
//! # Why this is a separate crate
//!
//! It drives [`probe_rs`] as a library, which has no place in a `no_std`
//! dependency. [`provision`] needs the library form: the NVM page buffer is
//! volatile, so its erase, buffer fill and quad-word commit must be one
//! uninterrupted sequence inside one attach, against a core that is not
//! itself driving NVMCTRL. A CLI that attaches and detaches per invocation
//! cannot hold that together.
//!
//! # Library or binary
//!
//! The library is the composable form, for a project's own `xtask` to call
//! alongside whatever else its build does. The `cli` feature, on by default,
//! adds the `samd5-boot-tools` binary for driving a part by hand.

pub mod flash;
pub mod image;
pub mod probe;
pub mod provision;

pub use probe::{Device, Mode};
pub use provision::{Fuses, UserPage};
