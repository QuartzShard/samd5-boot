//! Boot-info block: a fixed structure the BOOT binary embeds and the
//! application reads at runtime to audit BOOT/app compatibility. BOOT
//! installs it with [`install_boot_info!`](crate::install_boot_info) and
//! the linker pins it to the top of the BOOT region
//! ([`consts::BOOT_INFO_ADDR`]); the
//! application reads it there with [`read`].

use crate::consts;

/// Compatibility block the BOOT binary embeds at [`consts::BOOT_INFO_ADDR`]
/// for the application to audit at runtime. Check `magic` against [`MAGIC`]
/// before trusting the rest. Layout is APPEND ONLY (like the manifest), so an
/// older application still reads the fields it knows. `boot_size` is the
/// `BOOT_SIZE` this BOOT was built with, which the app compares against its
/// own; `build_id` identifies the exact BOOT build (e.g. a VCS/CI hash).
#[derive(bytemuck::AnyBitPattern, bytemuck::NoUninit, Clone, Copy)]
#[repr(C)]
pub struct BootInfo {
    pub magic: u32,
    pub abi_version: u16,
    pub transport_version: u16,
    pub boot_size: u32,
    pub build_id: u32,
}

/// Sentinel for a populated boot-info block ([`BootInfo::magic`]).
pub const MAGIC: u32 = 0xb007_1f0b;
/// Current boot-info layout version ([`BootInfo::abi_version`]).
pub const ABI_VERSION: u16 = 1;

// Must fit the page reserved for it at the top of the BOOT region.
const _: () = assert!(size_of::<BootInfo>() <= consts::PAGE_SIZE);

/// Read the boot-info block the running BOOT embedded, from its fixed
/// location at the top of the BOOT region.
pub fn read() -> BootInfo {
    // SAFETY: BOOT_INFO_ADDR is a fixed, always-populated flash location
    // holding a BootInfo (pinned by samd5_boot_boot.x).
    unsafe { (consts::BOOT_INFO_ADDR as *const BootInfo).read_volatile() }
}

/// Place the BOOT binary's boot-info block at the fixed location the
/// application reads it from
/// ([`consts::BOOT_INFO_ADDR`](crate::consts::BOOT_INFO_ADDR)): the top of
/// the BOOT region. Call once at item level in the BOOT binary; requires
/// linking with `samd5_boot_boot.x`, which pins the section.
#[macro_export]
macro_rules! install_boot_info {
    ($boot_info:expr) => {
        #[unsafe(link_section = ".samd5_boot_info")]
        #[used]
        static SAMD5_BOOT_INFO: $crate::boot_info::BootInfo = $boot_info;
    };
}
