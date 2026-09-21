//! Persistent bootloader storage. Used for counting boot trials, anti-rollback, etc
//! Includes the layout of stored data, and a trait to implement to provide a backend.
//!
//! Every sub-struct of [`BootStore`] is exactly one 32-bit word: the
//! unit a backend must write power-atomically, and each word has a
//! single writer (BOOT for [`BootState`] and [`Rollback`], the
//! application for [`UpdateMailbox`]). All-zeros decodes as the safe
//! fresh-chip default throughout.

use core::convert::Infallible;

use atsamd_hal::{nvm::PhysicalBank, pac::Nvmctrl};

use crate::consts::SEEPROM_ADDR;

/// The full persisted boot record: three single-word sub-structs, each
/// written independently and power-atomically (see [`BootStorage`]). Read at
/// boot and rewritten as state advances.
#[derive(bytemuck::AnyBitPattern, bytemuck::NoUninit, Clone, Copy, Default)]
#[repr(C)]
pub struct BootStore {
    pub boot_state: BootState,
    pub rollback: Rollback,
    pub mailbox: UpdateMailbox,
}

const _: () = assert!(
    size_of::<BootState>() == 4 && size_of::<Rollback>() == 4 && size_of::<UpdateMailbox>() == 4
);

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

    /// On ReadErr, you probably want to either retry or `unwrap_or_default()` (zeroed)
    fn read(&mut self) -> Result<BootStore, Self::ReadErr>;
    /// Persist the record. Must honour the per-word power-atomicity in the
    /// trait's `# Safety` contract: only whole words may change, never a torn
    /// one.
    fn write(&mut self, val: BootStore) -> Result<(), Self::WriteErr>;
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

    fn read(&mut self) -> Result<BootStore, Self::ReadErr> {
        Ok(BootStore::default())
    }

    fn write(&mut self, _: BootStore) -> Result<(), Self::WriteErr> {
        Ok(())
    }
}

const STORE_WORDS: usize = size_of::<BootStore>() / 4;

/// A word write to a locked SmartEEPROM is discarded silently; the
/// read-back in [`SmartEepromStore`]'s `write` surfaces it as this.
pub struct SeeWriteFailed;

/// Why [`SmartEepromStore::new`] rejected the live SmartEEPROM configuration.
/// `SeeUnavailable`: SBLK 0 (disabled) or a reserved SBLK (11+). `SeeBuffered`:
/// `SEECFG.WMODE` buffered, which defers word commits and voids per-word
/// power-atomicity. `OffsetOutOfRange`: `OFFSET + size_of::<BootStore>()` past
/// the configured virtual size.
pub enum StoreConfigError {
    SeeUnavailable,
    SeeLocked,
    SeeBuffered,
    OffsetOutOfRange,
}

/// [`BootStorage`] on SmartEEPROM. `OFFSET` is in bytes from the
/// start of the virtual space, so the record coexists with application
/// data stored elsewhere in it.
pub struct SmartEepromStore<const OFFSET: usize>(());

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
unsafe impl<const OFFSET: usize> BootStorage for SmartEepromStore<OFFSET> {
    type ReadErr = Infallible;
    type WriteErr = SeeWriteFailed;

    fn read(&mut self) -> Result<BootStore, Infallible> {
        let mut words = [0u32; STORE_WORDS];
        for (i, word) in words.iter_mut().enumerate() {
            Self::wait_ready();
            // SAFETY: in range per `new()`'s contract, BUSY just polled
            *word = unsafe { Self::word_ptr(i).read_volatile() };
        }
        Ok(bytemuck::cast(words))
    }

    fn write(&mut self, val: BootStore) -> Result<(), SeeWriteFailed> {
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
