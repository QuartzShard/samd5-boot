//! Boot-info block: a fixed structure the BOOT binary embeds and the
//! application reads at runtime to audit BOOT/app compatibility. BOOT
//! installs it with [`install_boot_info!`](crate::install_boot_info) and the
//! linker pins it to the top of the BOOT region
//! ([`consts::BOOT_INFO_ADDR`]); the application reads it there with
//! [`read`].

use crate::consts;

/// Compatibility block the BOOT binary embeds at [`consts::BOOT_INFO_ADDR`]
/// for the application to audit at runtime. [`read`] hands back whatever is
/// stored, so put it through [`BootInfo::validated`] before trusting the rest.
/// New fields are append only, as in the manifest, so an older application
/// still reads the fields it knows.
///
/// `boot_size` is the `BOOT_SIZE` this BOOT was built with; `build_id`
/// identifies the exact BOOT build (e.g. a VCS/CI hash). A BOOT and an
/// application built with different `bootprot-*` features do not agree on
/// where this block lives, so that mismatch shows up as
/// [`validated`](BootInfo::validated) finding nothing at all, not as an
/// unequal `boot_size`.
#[derive(bytemuck::AnyBitPattern, bytemuck::NoUninit, Clone, Copy)]
#[repr(C)]
pub struct BootInfo {
    pub magic: u32,
    pub abi_version: u16,
    /// What the BOOT binary's own download transport speaks. This crate
    /// never reads it; BOOT and the application agree on its meaning.
    pub transport_version: u16,
    pub boot_size: u32,
    pub build_id: u32,
}

/// Sentinel for a populated boot-info block ([`BootInfo::magic`])
pub const MAGIC: u32 = 0xb007_1f0b;
/// Current boot-info layout version ([`BootInfo::abi_version`])
pub const ABI_VERSION: u16 = 1;

impl BootInfo {
    /// The block as stored, or `None` if nothing populated it or it predates
    /// this build's [`ABI_VERSION`]
    pub const fn validated(self) -> Option<Self> {
        if self.magic == MAGIC && self.abi_version >= ABI_VERSION {
            Some(self)
        } else {
            None
        }
    }
}

// Must fit the page reserved for it at the top of the BOOT region.
const _: () = assert!(size_of::<BootInfo>() <= consts::PAGE_SIZE);

/// Read the boot-info block the running BOOT embedded, from its fixed
/// location at the top of the BOOT region
///
/// This dereferences a fixed absolute address and is meaningful only in a
/// binary running on the part.
pub fn read() -> BootInfo {
    // SAFETY: BOOT_INFO_ADDR is a fixed, 4-byte-aligned flash location that
    // samd5_boot_boot.x pins the block to, and every bit pattern is a valid
    // BootInfo, so an unprogrammed block reads as a record `validated`
    // rejects rather than as a fault.
    unsafe { (consts::BOOT_INFO_ADDR as *const BootInfo).read_volatile() }
}

/// Place the BOOT binary's boot-info block at the fixed location the
/// application reads it from ([`consts::BOOT_INFO_ADDR`]): the top of the
/// BOOT region. Call once at item level in the BOOT binary; requires linking
/// with `samd5_boot_boot.x`, which pins the section.
#[macro_export]
macro_rules! install_boot_info {
    ($boot_info:expr) => {
        #[unsafe(link_section = ".samd5_boot_info")]
        #[used]
        static SAMD5_BOOT_INFO: $crate::boot_info::BootInfo = $boot_info;
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::offset_of;

    /// The block is a fixed flash address an application reads out of a
    /// bootloader built separately from it, so the offsets are the contract.
    /// Growth is append-only into the reserved page; a field that moves
    /// needs [`ABI_VERSION`] raised.
    #[test]
    fn layout_is_pinned() {
        assert_eq!(size_of::<BootInfo>(), 16);
        assert_eq!(offset_of!(BootInfo, magic), 0);
        assert_eq!(offset_of!(BootInfo, abi_version), 4);
        assert_eq!(offset_of!(BootInfo, transport_version), 6);
        assert_eq!(offset_of!(BootInfo, boot_size), 8);
        assert_eq!(offset_of!(BootInfo, build_id), 12);
        assert!(size_of::<BootInfo>() <= consts::PAGE_SIZE);
    }
}
