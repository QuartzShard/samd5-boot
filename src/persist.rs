//! Persistent bootloader storage. Used for counting boot trials, anti-rollback, etc
//! Includes the layout of stored data, and a trait to implement to provide a backend.
//!
//! Every sub-struct of [`BootStore`] is exactly one 32-bit word: the unit a
//! backend must write power-atomically. The record carries a checksum over
//! those words, so a backing store that was never written, or a write torn
//! by a power cut, is detected and reported as the fresh default rather than
//! trusted. That matters because "unwritten" is not zeros everywhere: an
//! erased SmartEEPROM region reads as 0xFF, which would otherwise decode as
//! every mailbox flag set and a spent trial budget.

#[cfg(feature = "target")]
use core::convert::Infallible;

#[cfg(feature = "target")]
use atsamd_hal::{nvm::PhysicalBank, pac::Nvmctrl};

#[cfg(feature = "target")]
use crate::consts::SEEPROM_ADDR;

/// The full persisted boot record (see [`BootStorage`]). Read at boot and
/// rewritten as state advances.
#[derive(bytemuck::AnyBitPattern, bytemuck::NoUninit, Clone, Copy, Default)]
#[repr(C)]
pub struct BootStore {
    pub boot_state: BootState,
    pub rollback: Rollback,
    pub mailbox: UpdateMailbox,
    checksum: u32,
}

const _: () = assert!(
    size_of::<BootState>() == 4 && size_of::<Rollback>() == 4 && size_of::<UpdateMailbox>() == 4
);
const _: () = assert!(size_of::<BootStore>() == 16);

impl BootStore {
    /// Bytes the checksum covers: everything ahead of it.
    const COVERED: usize = size_of::<BootStore>() - size_of::<u32>();

    fn computed(&self) -> u32 {
        crc32(&bytemuck::bytes_of(self)[..Self::COVERED])
    }

    /// Stamp the checksum. A backend's `write` does this, so a record on its
    /// way to storage always carries a current one.
    pub fn seal(&mut self) {
        self.checksum = self.computed();
    }

    /// The record as stored, or `None` if the bytes do not check out.
    pub fn validated(self) -> Option<Self> {
        (self.checksum == self.computed()).then_some(self)
    }
}

/// CRC-32/ISO-HDLC, the same convention [`crate::crc32`] pins with test
/// vectors. Written table-free so BOOT, which otherwise never links that
/// table, does not gain 1 KiB of .rodata for a twelve-byte record.
fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &byte in bytes {
        crc ^= byte as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

/// Implementation for storing and retrieving `BootStore` from whatever persistent storage you have
/// in your system - SmartEEPROM backed impl provided.
///
/// # Safety
/// Each 32-bit word of [`BootStore`] (one sub-struct) must be written
/// atomically with respect to power loss: a cut mid-`write` may leave
/// any mix of old and new *words*, never a torn word. No ordering
/// between words is promised or required.
pub unsafe trait BootStorage {
    type WriteErr;
    type ReadErr;

    /// Hand back the stored bytes as they are, checksum unexamined.
    fn read_raw(&mut self) -> Result<BootStore, Self::ReadErr>;

    /// Persist the record. Must honour the per-word power-atomicity in the
    /// trait's `# Safety` contract: only whole words may change, never a torn
    /// one.
    fn write_raw(&mut self, val: BootStore) -> Result<(), Self::WriteErr>;

    /// Read the record, falling back to the fresh default when the stored
    /// bytes do not check out.
    ///
    /// This is the accessor to use; `read_raw` is the backend's hook.
    fn read(&mut self) -> Result<BootStore, Self::ReadErr> {
        Ok(self.read_raw()?.validated().unwrap_or_default())
    }

    /// Stamp the checksum and store the record.
    fn write(&mut self, mut val: BootStore) -> Result<(), Self::WriteErr> {
        val.seal();
        self.write_raw(val)
    }
}

/// Trial bookkeeping. Bank states are keyed by *physical* bank
/// (`STATUS.AFIRST` domain): the record survives BKSWRST while the
/// slot mapping flips, so resolve which field is active via
/// `Nvm::first_bank()`.
#[derive(bytemuck::AnyBitPattern, bytemuck::NoUninit, Clone, Copy, Default)]
#[repr(C)]
pub struct BootState {
    /// Bank A in bits 1:0, bank B in bits 3:2; 2 bits ↔ 4 variants is
    /// total, so decode is infallible. Upper bits reserved, preserved.
    states: u8,
    pub boot_count: u8,
    /// Why the last trial reverted; 0 = none. Codes land with the
    /// revert flow.
    pub revert_reason: u8,
    reserved: u8,
}

/// Keyed by the hal's `PhysicalBank`, so these are the one part of the
/// record that a host build cannot have.
#[cfg(feature = "target")]
impl BootState {
    const fn shift(bank: &PhysicalBank) -> u8 {
        match bank {
            PhysicalBank::A => 0,
            PhysicalBank::B => 2,
        }
    }

    /// The stored [`BankState`] of a physical bank.
    pub fn bank(self, bank: &PhysicalBank) -> BankState {
        BankState::from_bits(self.states >> Self::shift(bank))
    }

    /// Overwrite one physical bank's [`BankState`], leaving the other bank's
    /// bits and the reserved upper bits untouched.
    pub fn set_bank(&mut self, bank: &PhysicalBank, state: BankState) {
        let shift = Self::shift(bank);
        self.states = (self.states & !(0b11 << shift)) | ((state as u8) << shift);
    }

    /// At most one bank is ever `New`: marking one demotes a stale
    /// `New` on the other to `None`.
    pub fn mark_new(&mut self, bank: &PhysicalBank) {
        let other = match bank {
            PhysicalBank::A => PhysicalBank::B,
            PhysicalBank::B => PhysicalBank::A,
        };
        if let BankState::New = self.bank(&other) {
            self.set_bank(&other, BankState::None);
        }
        self.set_bank(bank, BankState::New);
    }
}

/// State of one bank's image, packed 2 bits per bank in [`BootState`] (never
/// stored directly). `None` is the all-zeros default, no image or a fresh chip,
/// bootable in steady state; `New` is an installed image on trial, counted
/// against the attempt budget until confirmed; `Valid` is a confirmed image;
/// `Invalid` is condemned and never re-trialed.
#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub enum BankState {
    #[default]
    None = 0,
    Valid = 1,
    New = 2,
    Invalid = 3,
}

impl BankState {
    #[cfg(feature = "target")]
    const fn from_bits(bits: u8) -> Self {
        match bits & 0b11 {
            0 => Self::None,
            1 => Self::Valid,
            2 => Self::New,
            3 => Self::Invalid,
            _ => unreachable!(),
        }
    }
}

/// Anti-rollback watermark
#[derive(bytemuck::AnyBitPattern, bytemuck::NoUninit, Clone, Copy, Default)]
#[repr(C)]
pub struct Rollback {
    pub highest_seen_manifest: u16,
    reserved: u16,
}

/// Application to BOOT signals: the app sets a flag at runtime, BOOT
/// reads it and clears it when it acts on it. All flags are edge-
/// triggered, so a set flag is consumed once.
#[derive(bytemuck::AnyBitPattern, bytemuck::NoUninit, Clone, Copy, Default)]
#[repr(C)]
pub struct UpdateMailbox {
    flags: u8,
    reserved: [u8; 3],
}

impl UpdateMailbox {
    const REQUEST: u8 = 1 << 0;
    const CONFIRM: u8 = 1 << 1;
    const REJECT: u8 = 1 << 2;

    fn get(self, bit: u8) -> bool {
        self.flags & bit != 0
    }

    fn set(&mut self, bit: u8, on: bool) {
        if on {
            self.flags |= bit;
        } else {
            self.flags &= !bit;
        }
    }

    /// Enter download mode on the next boot.
    pub fn request_update(self) -> bool {
        self.get(Self::REQUEST)
    }
    pub fn set_request_update(&mut self, on: bool) {
        self.set(Self::REQUEST, on)
    }

    /// The trial image ran; promote it on the next boot. Only acted on
    /// while the image is actually on trial.
    pub fn confirmed(self) -> bool {
        self.get(Self::CONFIRM)
    }
    pub fn set_confirmed(&mut self, on: bool) {
        self.set(Self::CONFIRM, on)
    }

    /// The application condemned the running image; revert on the next
    /// boot regardless of which state that image is in. When to set this
    /// is the application's call.
    pub fn rejected(self) -> bool {
        self.get(Self::REJECT)
    }
    pub fn set_rejected(&mut self, on: bool) {
        self.set(Self::REJECT, on)
    }
}

/// `revert_reason` codes
pub mod reason {
    pub const NONE: u8 = 0;
    pub const ATTEMPTS_EXHAUSTED: u8 = 1;
    pub const VERIFY_FAILED: u8 = 2;
    pub const APP_REJECTED: u8 = 3;
}

/// [`BootStorage`] for targets without SmartEEPROM: reads zeroed books,
/// discards writes. Trial bookkeeping is inert; every boot takes the
/// steady verify-and-jump path.
pub struct NoStore;

// SAFETY: no writes occur
unsafe impl BootStorage for NoStore {
    type ReadErr = core::convert::Infallible;
    type WriteErr = core::convert::Infallible;

    fn read_raw(&mut self) -> Result<BootStore, Self::ReadErr> {
        Ok(BootStore::default())
    }

    fn write_raw(&mut self, _: BootStore) -> Result<(), Self::WriteErr> {
        Ok(())
    }
}

/// [`BootStorage`] in backup RAM.
///
/// Backup RAM keeps its contents across any reset, including the one BKSWRST
/// performs, so the whole trial and rollback cycle works normally. It does
/// not survive loss of power: a cold boot reads as an unwritten store, which
/// the checksum turns into the fresh default. That makes this the right
/// backing for a bench rig, and for a product that is always powered and
/// wants a blackout treated as a blank slate; anything that must remember a
/// trial across a power cut needs [`SmartEepromStore`] or another
/// non-volatile backing.
///
/// `OFFSET` is in bytes from the base of backup RAM, so the record can sit
/// alongside whatever else the application keeps there.
#[cfg(feature = "target")]
pub struct BkupRamStore<const OFFSET: usize>(());

#[cfg(feature = "target")]
impl<const OFFSET: usize> BkupRamStore<OFFSET> {
    /// # Safety
    ///
    /// Nothing else may use `OFFSET..OFFSET + size_of::<BootStore>()` of
    /// backup RAM.
    pub const unsafe fn new() -> Self {
        const { assert!(OFFSET.is_multiple_of(4)) };
        const {
            assert!(OFFSET + size_of::<BootStore>() <= crate::consts::BKUPRAM_SIZE);
        }
        Self(())
    }

    fn ptr() -> *mut BootStore {
        (crate::consts::BKUPRAM_ADDR + OFFSET) as *mut BootStore
    }
}

// SAFETY: a 32-bit RAM write cannot tear, which is all the trait requires.
#[cfg(feature = "target")]
unsafe impl<const OFFSET: usize> BootStorage for BkupRamStore<OFFSET> {
    type ReadErr = Infallible;
    type WriteErr = Infallible;

    fn read_raw(&mut self) -> Result<BootStore, Infallible> {
        // SAFETY: in range per `new`, and every bit pattern is a valid
        // BootStore, so uninitialised backup RAM reads as a rejected record
        // rather than a fault.
        Ok(unsafe { Self::ptr().read_volatile() })
    }

    fn write_raw(&mut self, val: BootStore) -> Result<(), Infallible> {
        // SAFETY: as in `read_raw`.
        unsafe { Self::ptr().write_volatile(val) };
        Ok(())
    }
}

#[cfg(feature = "target")]
const STORE_WORDS: usize = size_of::<BootStore>() / 4;

/// A word write to a locked SmartEEPROM is discarded silently; the
/// read-back in [`SmartEepromStore`]'s `write` surfaces it as this.
#[cfg(feature = "target")]
pub struct SeeWriteFailed;

/// Why [`SmartEepromStore::new`] rejected the live SmartEEPROM configuration.
/// `SeeUnavailable`: SBLK 0 (disabled) or a reserved SBLK (11+). `SeeLocked`:
/// `SEESTAT.LOCK` set (see [`SeeWriteFailed`]). `SeeBuffered`: `SEECFG.WMODE`
/// buffered, which defers word commits and voids per-word power-atomicity.
/// `OffsetOutOfRange`: `OFFSET + size_of::<BootStore>()` past the configured
/// virtual size.
#[cfg(feature = "target")]
pub enum StoreConfigError {
    SeeUnavailable,
    SeeLocked,
    SeeBuffered,
    OffsetOutOfRange,
}

/// [`BootStorage`] on SmartEEPROM. `OFFSET` is in bytes from the
/// start of the virtual space, so the record coexists with application
/// data stored elsewhere in it.
#[cfg(feature = "target")]
pub struct SmartEepromStore<const OFFSET: usize>(());

#[cfg(feature = "target")]
impl<const OFFSET: usize> SmartEepromStore<OFFSET> {
    /// Validates the live SmartEEPROM configuration against `OFFSET`.
    ///
    /// Caller must ensure that nothing else is stored in
    /// `OFFSET..OFFSET + size_of::<BootStore>()`.
    pub fn new() -> Result<Self, StoreConfigError> {
        const { assert!(OFFSET.is_multiple_of(4)) };
        // SAFETY: read-only status access
        let regs = unsafe { &*Nvmctrl::ptr() };
        let seestat = regs.seestat().read();
        // Virtual size per DS Table 25-6: PSZ scales it, SBLK caps it.
        let cap = match seestat.sblk().bits() {
            0 | 11.. => return Err(StoreConfigError::SeeUnavailable),
            1 => 4096,
            2 => 8192,
            3 | 4 => 16384,
            5..=8 => 32768,
            9 | 10 => 65536,
        };
        let virtual_size = usize::min(512 << seestat.psz().bits(), cap);
        if OFFSET + size_of::<BootStore>() > virtual_size {
            return Err(StoreConfigError::OffsetOutOfRange);
        }
        if seestat.lock().bit_is_set() {
            return Err(StoreConfigError::SeeLocked);
        }
        if regs.seecfg().read().wmode().bit_is_set() {
            return Err(StoreConfigError::SeeBuffered);
        }
        Ok(Self(()))
    }

    fn word_ptr(index: usize) -> *mut u32 {
        (SEEPROM_ADDR + OFFSET + 4 * index) as *mut u32
    }

    fn wait_ready() {
        // SAFETY: read-only busy poll
        let regs = unsafe { &*Nvmctrl::ptr() };
        while regs.seestat().read().busy().bit_is_set() {}
    }
}

// SAFETY: in unbuffered mode each 32-bit SEE write is journaled by the
// EEPROM emulation providing per-word power-atomicity.
#[cfg(feature = "target")]
unsafe impl<const OFFSET: usize> BootStorage for SmartEepromStore<OFFSET> {
    type ReadErr = Infallible;
    type WriteErr = SeeWriteFailed;

    fn read_raw(&mut self) -> Result<BootStore, Infallible> {
        let mut words = [0u32; STORE_WORDS];
        for (i, word) in words.iter_mut().enumerate() {
            Self::wait_ready();
            // SAFETY: in range per `new()`'s contract, BUSY just polled
            *word = unsafe { Self::word_ptr(i).read_volatile() };
        }
        Ok(bytemuck::cast(words))
    }

    fn write_raw(&mut self, val: BootStore) -> Result<(), SeeWriteFailed> {
        let words: [u32; STORE_WORDS] = bytemuck::cast(val);
        for (i, &word) in words.iter().enumerate() {
            let ptr = Self::word_ptr(i);
            Self::wait_ready();
            // Unchanged words are skipped: no wear or time spent
            // rewriting an identical journal entry.
            // SAFETY: as in `read`
            if unsafe { ptr.read_volatile() } == word {
                continue;
            }
            unsafe { ptr.write_volatile(word) };
            Self::wait_ready();
            if unsafe { ptr.read_volatile() } != word {
                return Err(SeeWriteFailed);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::crc32;

    /// Host tooling seals a record the target then validates, so this has to
    /// stay byte-identical to the image convention [`crate::crc32`] pins.
    #[test]
    fn record_crc_matches_the_image_convention() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        let record: [u8; 12] = [
            0x02, 0x03, 0x01, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
        ];
        assert_eq!(crc32(&record), crate::crc32::crc32(&record));
    }
}
