//! Family-wide geometry (DS60001507, §25 NVMCTRL) is defined
//! unconditionally; flash/RAM density is selected by the part feature.
//! Exactly one part feature must be enabled: none is a compile error
//! below, and more than one collides on the density const definitions.
//!
//! Bank/slot vocabulary is defined in the crate docs.

pub const FLASH_ADDR: usize = 0x0000_0000;
pub const PAGE_SIZE: usize = 512;
pub const PAGE_SIZE_WORDS: usize = PAGE_SIZE / 4;
pub const PAGES_PER_BLOCK: usize = 16;
pub const ERASE_BLOCK_SIZE: usize = PAGE_SIZE * PAGES_PER_BLOCK;

/// Quad-word: granularity of the WQW program command.
pub const WRITE_UNIT: usize = 16;
pub const ERASED: u32 = 0xFFFFFFFF;

/// Fixed regardless of density, BOOTPROT, or SmartEEPROM settings.
pub const LOCK_REGION_COUNT: usize = 32;

/// BOOTPROT protects `(15 − BOOTPROT) ×` this from the base of flash.
pub const BOOTPROT_GRANULE: usize = 8 * 1024;
pub const BOOTPROT_MAX: usize = 15 * BOOTPROT_GRANULE;

/// All fuses (BOOTPROT, SEE, WDT, region locks) in one erase-rewrite unit.
pub const USER_PAGE_ADDR: usize = 0x0080_4000;
pub const USER_PAGE_SIZE: usize = 512;

pub const SEEPROM_ADDR: usize = 0x4400_0000;
pub const RAM_ADDR: usize = 0x2000_0000;
pub const BKUPRAM_ADDR: usize = 0x4700_0000;
pub const BKUPRAM_SIZE: usize = 8 * 1024;

// ── Geometry formulas ───────────────────────────────────────────────────

/// The derivations below, over a flash size and BOOT size supplied by the
/// caller rather than selected by a feature. The firmware binds them to its
/// part at compile time; host tooling (`xtask`) applies them to a part named
/// on the command line, so a fuse encoding has exactly one definition.
pub mod geometry {
    use super::{BOOTPROT_GRANULE, BOOTPROT_MAX, LOCK_REGION_COUNT, PAGE_SIZE};

    pub const fn bank_size(flash_size: usize) -> usize {
        flash_size / 2
    }

    pub const fn lock_region_size(flash_size: usize) -> usize {
        flash_size / LOCK_REGION_COUNT
    }

    /// Fuse encoding of a BOOT size: BOOTPROT protects `(15 - value)` times
    /// [`BOOTPROT_GRANULE`], so the field counts down.
    pub const fn bootprot_value(boot_size: usize) -> u8 {
        (15 - boot_size / BOOTPROT_GRANULE) as u8
    }

    /// Lock-region bits covering both copies of BOOT: the first `boot_size`
    /// of each bank. Set bits select the BOOT regions; NVM LOCKS and RUNLOCK
    /// invert that (a clear bit locks), so a caller clears these.
    pub const fn boot_region_mask(flash_size: usize, boot_size: usize) -> u32 {
        let region = lock_region_size(flash_size);
        let per_boot = (boot_size / region) as u32;
        let inactive_first = (bank_size(flash_size) / region) as u32;
        ((1u32 << per_boot) - 1) * (1 | (1 << inactive_first))
    }

    /// Where the boot-info block sits for a given BOOT size: the top page of
    /// the BOOT region.
    pub const fn boot_info_addr(boot_size: usize) -> usize {
        super::FLASH_ADDR + boot_size - PAGE_SIZE
    }

    /// Whether a BOOT size is usable on a part of this density. A BOOT region
    /// must be a whole number of lock regions (so it can be locked), must fit
    /// in a bank alongside an application, and cannot exceed what BOOTPROT
    /// can express.
    pub const fn boot_size_valid(flash_size: usize, boot_size: usize) -> bool {
        let region = lock_region_size(flash_size);
        boot_size > 0
            && boot_size <= BOOTPROT_MAX
            && boot_size < bank_size(flash_size)
            && boot_size % region == 0
            && boot_size % BOOTPROT_GRANULE == 0
    }
}

// ── Density selection (`density` cfg emitted by build.rs) ───────────────

#[cfg(density = "18")]
mod density {
    pub const FLASH_SIZE: usize = 256 * 1024;
    pub const RAM_SIZE: usize = 128 * 1024;
}

#[cfg(density = "19")]
mod density {
    pub const FLASH_SIZE: usize = 512 * 1024;
    pub const RAM_SIZE: usize = 192 * 1024;
}

#[cfg(density = "20")]
mod density {
    pub const FLASH_SIZE: usize = 1024 * 1024;
    pub const RAM_SIZE: usize = 256 * 1024;
}

#[cfg(all(
    feature = "target",
    not(any(density = "18", density = "19", density = "20"))
))]
compile_error!("samd5-boot: select exactly one chip feature (full part number, e.g. `samd51j20a`)");

#[cfg(any(density = "18", density = "19", density = "20"))]
pub use density::{FLASH_SIZE, RAM_SIZE};

// ── Derived geometry ────────────────────────────────────────────────────

/// Base of the active slot: the mapped-low bank, always at the flash base.
pub const ACTIVE_SLOT_ADDR: usize = FLASH_ADDR;

/// One physical bank: half the flash. A bank maps to either slot; the size
/// is the same in both roles.
#[cfg(any(density = "18", density = "19", density = "20"))]
pub const BANK_SIZE: usize = geometry::bank_size(FLASH_SIZE);
/// Base of the inactive slot: the mapped-high bank, in the upper half. The
/// download target and the swap destination.
#[cfg(any(density = "18", density = "19", density = "20"))]
pub const INACTIVE_SLOT_ADDR: usize = FLASH_ADDR + BANK_SIZE;
#[cfg(any(density = "18", density = "19", density = "20"))]
pub const BLOCKS_PER_BANK: usize = BANK_SIZE / ERASE_BLOCK_SIZE;
#[cfg(any(density = "18", density = "19", density = "20"))]
pub const FLASH_PAGES: usize = FLASH_SIZE / PAGE_SIZE;
#[cfg(any(density = "18", density = "19", density = "20"))]
pub const LOCK_REGION_SIZE: usize = geometry::lock_region_size(FLASH_SIZE);

// ── BOOT region (`boot_size` cfg emitted by build.rs) ───────────────────

#[cfg(all(boot_size = "16k", density = "20"))]
compile_error!(
    "bootprot-16k is invalid on 1 MiB (...20a) parts: their lock regions \
     are 32 KiB, so BOOT must be at least 32 KiB"
);

#[cfg(boot_size = "16k")]
pub const BOOT_SIZE: usize = 16 * 1024;
#[cfg(boot_size = "32k")]
pub const BOOT_SIZE: usize = 32 * 1024;
#[cfg(boot_size = "64k")]
pub const BOOT_SIZE: usize = 64 * 1024;
#[cfg(boot_size = "96k")]
pub const BOOT_SIZE: usize = 96 * 1024;

/// Fuse encoding of [`BOOT_SIZE`]: the value the BOOTPROT field must hold.
pub const BOOTPROT_VALUE: u8 = geometry::bootprot_value(BOOT_SIZE);

// The 16k + density-20 case is already reported by the compile_error
// above; skip the generic assert there so it is not a second error.
#[cfg(all(
    any(density = "18", density = "19", density = "20"),
    not(all(boot_size = "16k", density = "20"))
))]
const _: () = assert!(geometry::boot_size_valid(FLASH_SIZE, BOOT_SIZE));

/// Fixed location of the boot-info block: the top page of the BOOT
/// region. BOOT pins [`BootInfo`](crate::boot_info::BootInfo) here (via
/// `samd5_boot_boot.x`) and the application reads it at this absolute
/// address. Derived from BOOT_SIZE so BOOT and app agree (both build with
/// the same bootprot-* feature); a whole page is reserved so append-only
/// ABI growth never moves the address.
pub const BOOT_INFO_ADDR: usize = geometry::boot_info_addr(BOOT_SIZE);

/// Lock-region bits covering both copies of BOOT; a caller clears these.
#[cfg(any(density = "18", density = "19", density = "20"))]
pub const BOOT_REGIONS: u32 = geometry::boot_region_mask(FLASH_SIZE, BOOT_SIZE);

/// Offset of the [`AppManifest`](crate::manifest::AppManifest) within the app
/// region: past the largest vector table on this family (16 system exceptions +
/// 137 peripheral IRQs, the assert below), on the 1 KiB VTOR granule. BOOT
/// reads the manifest at `app_begin + MANIFEST_OFFSET`; the app links it there
/// via `samd5_boot_app.x`.
pub const MANIFEST_OFFSET: usize = 0x400;

const _: () = assert!(MANIFEST_OFFSET >= (16 + 137) * 4);

#[cfg(test)]
mod tests {
    use super::geometry::*;

    const K: usize = 1024;

    /// DS 25.6.14: BOOTPROT protects `(15 - value)` times 8 KiB. A wrong
    /// encoding here either leaves BOOT writable or protects into the
    /// application.
    #[test]
    fn bootprot_encoding_counts_down() {
        assert_eq!(bootprot_value(0), 15);
        assert_eq!(bootprot_value(16 * K), 13);
        assert_eq!(bootprot_value(32 * K), 11);
        assert_eq!(bootprot_value(64 * K), 7);
        assert_eq!(bootprot_value(96 * K), 3);
        assert_eq!(bootprot_value(super::BOOTPROT_MAX), 0);
    }

    /// Always 32 regions across the whole flash, whatever the density, so a
    /// region is a different size on every part and a bank is always 16 of
    /// them.
    #[test]
    fn regions_scale_with_density() {
        assert_eq!(lock_region_size(1024 * K), 32 * K);
        assert_eq!(lock_region_size(512 * K), 16 * K);
        assert_eq!(lock_region_size(256 * K), 8 * K);
        for flash in [256 * K, 512 * K, 1024 * K] {
            assert_eq!(bank_size(flash) / lock_region_size(flash), 16);
        }
    }

    /// Both copies of BOOT: the low regions of each bank, the inactive one
    /// starting at region 16.
    #[test]
    fn boot_mask_covers_both_banks() {
        assert_eq!(boot_region_mask(1024 * K, 32 * K), 0x0001_0001);
        assert_eq!(boot_region_mask(1024 * K, 64 * K), 0x0003_0003);
        assert_eq!(boot_region_mask(512 * K, 32 * K), 0x0003_0003);
        assert_eq!(boot_region_mask(256 * K, 16 * K), 0x0003_0003);
    }

    /// A BOOT region has to be a whole number of lock regions, or the mirror
    /// cannot be write protected. That is what rules out 16 KiB on a 1 MiB
    /// part, whose regions are 32 KiB.
    #[test]
    fn boot_size_must_be_lockable() {
        assert!(!boot_size_valid(1024 * K, 16 * K));
        assert!(boot_size_valid(1024 * K, 32 * K));
        assert!(boot_size_valid(512 * K, 16 * K));
        assert!(boot_size_valid(256 * K, 16 * K));
        // Past what BOOTPROT can express, and a BOOT that leaves no bank
        // for the application (128 KiB is the whole bank on a 256 KiB part).
        assert!(!boot_size_valid(1024 * K, 128 * K));
        assert!(!boot_size_valid(256 * K, 128 * K));
        assert!(!boot_size_valid(1024 * K, 0));
        // 96 KiB on a 256 KiB part is legal, if cramped: 32 KiB of app left.
        assert!(boot_size_valid(256 * K, 96 * K));
    }

    #[test]
    fn boot_info_sits_in_the_top_page() {
        assert_eq!(boot_info_addr(32 * K), 32 * K - super::PAGE_SIZE);
        assert_eq!(boot_info_addr(64 * K), 64 * K - super::PAGE_SIZE);
    }
}
