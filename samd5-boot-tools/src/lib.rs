//! Bench and manufacturing operations for a samd5-boot device, over a debug
//! probe.
//!
//! These are the parts of getting a bootloader onto a part that need a probe
//! rather than a compiler: writing the fuses that make the bootloader
//! protected, placing it at the head of both banks, and stamping an image so
//! the bootloader will accept it. The layouts and encodings all come from
//! [`samd5_boot`] itself, so a device programmed by this agrees with the
//! firmware by construction rather than by two implementations matching.
//!
//! # Why this is a separate crate
//!
//! It drives [`probe_rs`] as a library, which has no place in a `no_std`
//! dependency. Driving it as a library rather than shelling out to its CLI is
//! not a preference: the CLI attaches and detaches per invocation and never
//! halts the core, and the NVM page buffer is volatile, so an erase, a buffer
//! fill and the commit that follows have to be one uninterrupted sequence
//! against a core that is not itself driving NVMCTRL.
//!
//! # Two shapes
//!
//! The library is the composable form, for a project's own `xtask` to call
//! alongside whatever else its build does. The `cli` feature (on by default)
//! adds a `samd5-boot-tools` binary for driving a part by hand.

pub mod flash;
pub mod image;
pub mod probe;
pub mod provision;

pub use probe::{Device, Mode};
pub use provision::{Fuses, UserPage};
