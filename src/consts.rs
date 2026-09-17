//! Family-wide geometry (DS60001507, §25 NVMCTRL) is defined
//! unconditionally; flash/RAM density is selected by the part feature.
//! Exactly one part feature must be enabled: none is a compile error
//! below, and more than one collides on the density const definitions.
//!
//! Vocabulary: a *bank* is a physical flash bank (A/B, the
//! `STATUS.AFIRST` domain); a *slot* is a mapped position: the active
//! slot is always at the base of flash, the inactive slot in the upper
//! half. BKSWRST changes which bank occupies which slot.

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

#[cfg(not(any(density = "18", density = "19", density = "20")))]
compile_error!("samd5-boot: select exactly one chip feature (full part number, e.g. `samd51j20a`)");

// Stub so the compile_error above is the only diagnostic when no chip
// feature is selected.
#[cfg(not(any(density = "18", density = "19", density = "20")))]
mod density {
    pub const FLASH_SIZE: usize = 0;
    pub const RAM_SIZE: usize = 0;
}

pub use density::{FLASH_SIZE, RAM_SIZE};

// ── Derived geometry ────────────────────────────────────────────────────

pub const BANK_SIZE: usize = FLASH_SIZE / 2;
pub const ACTIVE_SLOT_ADDR: usize = FLASH_ADDR;
pub const INACTIVE_SLOT_ADDR: usize = FLASH_ADDR + BANK_SIZE;
pub const BLOCKS_PER_BANK: usize = BANK_SIZE / ERASE_BLOCK_SIZE;
pub const FLASH_PAGES: usize = FLASH_SIZE / PAGE_SIZE;
pub const LOCK_REGION_SIZE: usize = FLASH_SIZE / LOCK_REGION_COUNT;

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
pub const BOOTPROT_VALUE: u8 = (15 - BOOT_SIZE / BOOTPROT_GRANULE) as u8;

// The 16k + density-20 case is already reported by the compile_error
// above; skip the generic assert there so it is not a second error.
#[cfg(all(
    any(density = "18", density = "19", density = "20"),
    not(all(boot_size = "16k", density = "20"))
))]
const _: () = assert!(BOOT_SIZE < BANK_SIZE && BOOT_SIZE.is_multiple_of(LOCK_REGION_SIZE));

// Place after vector table
pub const MANIFEST_OFFSET: usize = 0x400;

const _: () = assert!(MANIFEST_OFFSET >= (16 + 137) * 4);
