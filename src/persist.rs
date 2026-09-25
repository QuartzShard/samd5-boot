//! Persistent boot record and the backend trait that stores it
//!
//! [`BootStore`] is the record: trial bookkeeping ([`BootState`]), the
//! application-to-BOOT mailbox ([`UpdateMailbox`]), and an anti-rollback
//! watermark ([`Rollback`]) that nothing reads yet. Implement
//! [`BootStorage`] to back it, or take one of the backends here:
//! `SmartEepromStore` (non-volatile), `BkupRamStore` (survives every reset
//! but not a power cut), [`Fallback`] (one covering for the other) and
//! [`NoStore`] (no persistence at all). The first two need the `target`
//! feature.
//!
//! Every sub-struct of [`BootStore`] is exactly one 32-bit word: the unit a
//! backend must write power-atomically. The record carries a checksum over
//! those words, so a backing store that was never written, or a write torn
//! by a power cut, is detected and reported as the fresh default rather than
//! trusted. That matters because "unwritten" is not zeros everywhere: an
//! erased SmartEEPROM region reads as 0xFF, which would otherwise decode as
//! every mailbox flag set and a spent trial budget.
//!
//! A record that fails that check reads as a fresh chip, not as the previous
//! record: a trial in progress is forgotten and the image that was on trial
//! boots as a steady image.

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
    /// Layout of the stored record, written into every sealed record and
    /// required of every record read back.
    ///
    /// The checksum cannot carry this: it covers whatever sixteen bytes are
    /// there and so validates an old record read under a new layout just as
    /// happily. A bootloader can be replaced in the field while a record
    /// outlives it in backup RAM or SmartEEPROM, and the record is what
    /// decides which bank boots, so reading one under the wrong layout is
    /// not a tolerable failure.
    ///
    /// Bump it only for a change an older reader cannot survive. Additions
    /// that an older reader can ignore (a new mailbox flag, the reserved
    /// fields, the spare bits of `states`) leave it alone, which is what
    /// keeps a bootloader able to read records written by newer ones.
    pub const FMT: u8 = 0;

    /// Bytes the checksum covers: everything ahead of it
    const COVERED: usize = size_of::<BootStore>() - size_of::<u32>();

    fn computed(&self) -> u32 {
        crc32(&bytemuck::bytes_of(self)[..Self::COVERED])
    }

    /// Stamp the checksum
    ///
    /// [`BootStorage::write`] does this, so a record on its way to storage
    /// always carries a current one. Call it directly only when writing a
    /// record through something other than that method.
    pub fn seal(&mut self) {
        self.boot_state.fmt = Self::FMT;
        self.checksum = self.computed();
    }

    /// The record as stored, or `None` when it is not one this build can act
    /// on: the bytes do not check out, or they are a layout it does not
    /// know.
    ///
    /// Rejecting an unknown layout is what makes [`BootStore::FMT`] work.
    /// A reader that ignored the field could not tell a future record from a
    /// current one, so the check has to be here from the first version
    /// rather than added alongside the first bump.
    ///
    /// [`BootStorage::read`] turns `None` into the fresh default, so the
    /// outcome is a part that starts over rather than one acting on a record
    /// it has misread.
    pub fn validated(self) -> Option<Self> {
        (self.boot_state.fmt == Self::FMT && self.checksum == self.computed()).then_some(self)
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

/// Backend that stores and retrieves the [`BootStore`] record
///
/// Implement it over whatever persistent storage the system has; the
/// SmartEEPROM and backup RAM backends below are provided.
///
/// # Safety
///
/// Each 32-bit word of [`BootStore`] (one sub-struct) must be written
/// atomically with respect to power loss: a cut mid-[`write`](Self::write)
/// may leave any mix of old and new *words*, never a torn word. No
/// ordering between words is promised or required.
pub unsafe trait BootStorage {
    type WriteErr;
    type ReadErr;

    /// Hand back the stored bytes as they are, checksum unexamined
    fn read_raw(&mut self) -> Result<BootStore, Self::ReadErr>;

    /// Persist the record. Must honour the per-word power-atomicity in the
    /// trait's `# Safety` contract: only whole words may change, never a torn
    /// one.
    fn write_raw(&mut self, val: BootStore) -> Result<(), Self::WriteErr>;

    /// Read the record, falling back to the fresh default when the stored
    /// bytes do not check out
    ///
    /// This is the accessor to use; [`read_raw`](Self::read_raw) is the
    /// backend's hook.
    fn read(&mut self) -> Result<BootStore, Self::ReadErr> {
        Ok(self.read_raw()?.validated().unwrap_or_default())
    }

    /// Stamp the checksum and store the record
    fn write(&mut self, mut val: BootStore) -> Result<(), Self::WriteErr> {
        val.seal();
        self.write_raw(val)
    }
}

/// Trial bookkeeping. Bank states are keyed by *physical* bank
/// (`STATUS.AFIRST` domain): the record survives BKSWRST while the
/// slot mapping flips, so resolve which field is active via
/// `Nvm::first_bank()`.
///
/// The per-bank accessors are keyed by the hal's `PhysicalBank`, so they
/// exist only under the `target` feature.
#[derive(bytemuck::AnyBitPattern, bytemuck::NoUninit, Clone, Copy, Default)]
#[repr(C)]
pub struct BootState {
    /// Bank A in bits 1:0, bank B in bits 3:2; 2 bits to 4 variants is
    /// total, so decode is infallible. Bits 7:4 are reserved: a writer
    /// leaves them as it found them, a reader ignores them.
    states: u8,
    pub boot_count: u8,
    /// Read it typed with [`BootState::reason`]
    pub revert_reason: u8,
    /// Layout of the record these bytes belong to, [`BootStore::FMT`].
    fmt: u8,
}

impl BootState {
    /// Typed view of [`revert_reason`](Self::revert_reason). `None` is a code
    /// this build cannot decode, not [`RevertReason::None`].
    pub const fn reason(&self) -> Option<RevertReason> {
        RevertReason::from_u8(self.revert_reason)
    }

    pub fn set_reason(&mut self, reason: RevertReason) {
        self.revert_reason = reason as u8;
    }
}

#[cfg(feature = "target")]
impl BootState {
    const fn shift(bank: &PhysicalBank) -> u8 {
        match bank {
            PhysicalBank::A => 0,
            PhysicalBank::B => 2,
        }
    }

    /// The stored [`BankState`] of a physical bank
    pub fn bank(self, bank: &PhysicalBank) -> BankState {
        BankState::from_bits(self.states >> Self::shift(bank))
    }

    /// Overwrite one physical bank's [`BankState`], leaving the other bank's
    /// bits and the reserved upper bits untouched
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
/// stored directly). `None` is the all-zeros default, no image or a fresh
/// chip, bootable in steady state; `New` is an installed image on trial,
/// counted against the attempt budget until confirmed; `Valid` is a
/// confirmed image.
///
/// `Invalid` is condemned: the bootloader never boots or re-trials it,
/// though installing a new image over that bank resets it to `New`.
#[repr(u8)]
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
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
            _ => Self::Invalid,
        }
    }
}

/// Anti-rollback watermark, reserved: nothing in this crate reads or writes
/// it yet.
///
/// The intended contract is the highest
/// [`version`](crate::manifest::ManifestBody::version) this part
/// has accepted, so an image that names a lower one can be refused. It is
/// carried now so adopting it later needs no [`BootStore::FMT`] bump: a
/// reader of this version leaves the field alone rather than zeroing it.
#[derive(bytemuck::AnyBitPattern, bytemuck::NoUninit, Clone, Copy, Default)]
#[repr(C)]
pub struct Rollback {
    pub highest_seen_manifest: u16,
    /// Reserved. Writers leave it as they found it, readers ignore it.
    reserved: u16,
}

/// Application to BOOT signals: the app sets a flag at runtime, BOOT
/// reads it and clears it when it acts on it. All flags are edge-
/// triggered, so a set flag is consumed once.
#[derive(bytemuck::AnyBitPattern, bytemuck::NoUninit, Clone, Copy, Default)]
#[repr(C)]
pub struct UpdateMailbox {
    /// Bits 2:0 are the three flags below. Bits 7:3 are reserved: a writer
    /// leaves them as it found them, a reader ignores them, so a flag added
    /// later costs no [`BootStore::FMT`] bump.
    flags: u8,
    /// Reserved. Writers leave it as they found it, readers ignore it.
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

    /// Whether the application asked BOOT to enter download mode on the
    /// next boot
    pub fn request_update(self) -> bool {
        self.get(Self::REQUEST)
    }
    pub fn set_request_update(&mut self, on: bool) {
        self.set(Self::REQUEST, on)
    }

    /// Whether the application marked the running image good. BOOT promotes
    /// it on the next boot, and only while it is still on trial.
    pub fn confirmed(self) -> bool {
        self.get(Self::CONFIRM)
    }
    pub fn set_confirmed(&mut self, on: bool) {
        self.set(Self::CONFIRM, on)
    }

    /// Whether the application condemned the running image. BOOT reverts on
    /// the next boot whatever state that image is in. When to set it is the
    /// application's call.
    pub fn rejected(self) -> bool {
        self.get(Self::REJECT)
    }
    pub fn set_rejected(&mut self, on: bool) {
        self.set(Self::REJECT, on)
    }
}

/// Why the bootloader last abandoned an image, the typed form of the raw
/// [`BootState::revert_reason`] byte. `None` is the steady state: nothing
/// has been reverted, or a later trial image was promoted, which is the only
/// thing that clears the code.
#[repr(u8)]
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub enum RevertReason {
    #[default]
    None = 0,
    AttemptsExhausted = 1,
    VerifyFailed = 2,
    AppRejected = 3,
}

impl RevertReason {
    /// Decode a stored [`BootState::revert_reason`] byte. `None` is a code
    /// this build does not know, which a newer BOOT may have written.
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::None),
            1 => Some(Self::AttemptsExhausted),
            2 => Some(Self::VerifyFailed),
            3 => Some(Self::AppRejected),
            _ => None,
        }
    }
}

/// [`BootStorage`] that persists nothing: reads a zeroed record, discards
/// writes. Trial bookkeeping is inert; every boot takes the steady
/// verify-and-jump path.
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

/// A store that keeps tracking when its primary cannot be written.
///
/// Writes go to `primary`. One that fails goes to `spare` instead, which is
/// then the authority until a primary write lands again or the spare itself
/// is lost. Reads prefer the spare exactly when it holds a sealed record,
/// which is the case where the primary is behind; a spare that cannot be
/// read counts as empty, so its read error never masks the primary.
///
/// The spare this exists for is `BkupRamStore`: it survives every reset
/// including the one `BKSWRST` performs, so a trial keeps counting and can
/// still revert, and it does *not* survive loss of power, so it cannot
/// outlive the fault it covers for. Without a spare, a primary whose writes
/// fail while an image is on trial leaves the bootloader with nowhere to go,
/// because installing a replacement needs a write too.
pub struct Fallback<P, S> {
    primary: P,
    spare: S,
}

impl<P, S> Fallback<P, S> {
    pub const fn new(primary: P, spare: S) -> Self {
        Self { primary, spare }
    }

    pub fn free(self) -> (P, S) {
        (self.primary, self.spare)
    }
}

// SAFETY: both backends carry the per-word atomicity contract, and a record
// is only ever handed to one of them whole.
unsafe impl<P: BootStorage, S: BootStorage> BootStorage for Fallback<P, S> {
    type ReadErr = P::ReadErr;
    /// The spare's, because a write is only lost when the spare loses it too
    type WriteErr = S::WriteErr;

    fn read_raw(&mut self) -> Result<BootStore, Self::ReadErr> {
        // A sealed spare means a primary write has failed since the last one
        // that landed. Validating here picks the source; the `read` above
        // validates again to decide whether to start from the fresh default,
        // which is a different question about a record already chosen.
        if let Ok(spare) = self.spare.read_raw()
            && spare.validated().is_some()
        {
            return Ok(spare);
        }
        self.primary.read_raw()
    }

    fn write_raw(&mut self, val: BootStore) -> Result<(), Self::WriteErr> {
        // Invalidate the spare first. Clearing it after a successful primary
        // write leaves a window in which a reset finds a spare that is older
        // than the primary and believes it. An unsealed record is what
        // "nothing here" looks like, so the default does the clearing.
        let _ = self.spare.write_raw(BootStore::default());
        if self.primary.write_raw(val).is_ok() {
            return Ok(());
        }
        self.spare.write_raw(val)
    }
}

/// [`BootStorage`] in backup RAM
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
    ///
    /// `OFFSET` must be a multiple of 4 and the record must fit within
    /// [`BKUPRAM_SIZE`](crate::consts::BKUPRAM_SIZE); both are checked at
    /// compile time.
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

/// Bound on the `INTFLAG.SEEWRC` spin in [`SmartEepromStore`], in loop
/// passes. Sized so that reaching it means the flag is not coming rather
/// than that a write is merely slow: a sector reallocation is the longest
/// thing a write can set off, and this is far more than one takes at any
/// core clock the part runs.
#[cfg(feature = "target")]
const WRITE_COMPLETE_SPINS: u32 = 1_000_000;

/// A word write to a locked SmartEEPROM is discarded silently; the
/// read-back in [`SmartEepromStore`]'s
/// [`write_raw`](BootStorage::write_raw) surfaces it as this.
#[cfg(feature = "target")]
pub struct SeeWriteFailed;

/// Why [`SmartEepromStore::new`] rejected the live SmartEEPROM
/// configuration. Each variant's condition is listed under that function's
/// `# Errors`.
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
    /// Validate the live SmartEEPROM configuration against `OFFSET`
    ///
    /// `OFFSET` is a byte offset into the virtual space and must be a
    /// multiple of 4; that is checked at compile time. Nothing else may be
    /// stored in `OFFSET..OFFSET + size_of::<BootStore>()`: application data
    /// written over the record is not detected, it just fails the checksum.
    ///
    /// # Errors
    ///
    /// * Returns [`StoreConfigError::SeeUnavailable`] if `Nvmctrl.SEESTAT`
    ///   `SBLK` is 0 (SmartEEPROM disabled) or 11 or higher (reserved).
    /// * Returns [`StoreConfigError::OffsetOutOfRange`] if the record does
    ///   not fit below the configured virtual size.
    /// * Returns [`StoreConfigError::SeeLocked`] if `Nvmctrl.SEESTAT.LOCK`
    ///   is set; writes would then be discarded silently (see
    ///   [`SeeWriteFailed`]).
    /// * Returns [`StoreConfigError::SeeBuffered`] if `Nvmctrl.SEECFG.WMODE`
    ///   selects buffered mode, which defers word commits and voids the
    ///   per-word power-atomicity [`BootStorage`] requires.
    pub fn new() -> Result<Self, StoreConfigError> {
        const { assert!(OFFSET.is_multiple_of(4)) };
        // SAFETY: read-only status access
        let regs = unsafe { &*Nvmctrl::ptr() };
        let seestat = regs.seestat().read();
        // Virtual size per DS60001507 table 25-6: PSZ scales it, SBLK caps it.
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

    /// Arm the completion flag so the wait after a write sees that write.
    ///
    /// `INTFLAG.SEEWRC` is write-one-to-clear and latches until cleared, so
    /// a flag left set by an earlier write would satisfy the next wait
    /// immediately.
    fn arm_write_complete() {
        // SAFETY: a write-one-to-clear of one flag; the other flags take a
        // zero, which leaves them alone.
        let regs = unsafe { &*Nvmctrl::ptr() };
        regs.intflag().write(|w| w.seewrc().set_bit());
    }

    /// Wait for a write to land.
    ///
    /// `SEESTAT.BUSY` alone is not enough. It reports the SmartEEPROM
    /// controller busy, which can read clear in the window between an AHB
    /// write being accepted and its journal entry being committed;
    /// `INTFLAG.SEEWRC` (DS 25.6.8.6) is the flag that says the write itself
    /// completed. Both are waited on: the completion first, then BUSY for
    /// any sector reallocation the write set off.
    ///
    /// The completion spin is bounded, and deliberately. The flag's exact
    /// behaviour has not been confirmed on silicon here, and a bootloader
    /// that spun forever on a flag that never arrives would be worse than
    /// one that fell through to the checks below. Exhausting the bound
    /// leaves the outcome to the same `BUSY` wait and read-back this had
    /// before, so the worst case is the old behaviour rather than a hang.
    fn wait_write_complete() {
        // SAFETY: read-only flag polls
        let regs = unsafe { &*Nvmctrl::ptr() };
        let mut spins = 0u32;
        while regs.intflag().read().seewrc().bit_is_clear() && spins < WRITE_COMPLETE_SPINS {
            spins += 1;
        }
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
            Self::arm_write_complete();
            unsafe { ptr.write_volatile(word) };
            Self::wait_write_complete();
            if unsafe { ptr.read_volatile() } != word {
                return Err(SeeWriteFailed);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A backend whose writes can be switched off, standing in for a store
    /// that has started refusing them.
    struct Flaky {
        held: BootStore,
        writable: bool,
    }

    // SAFETY: a test double over a plain field; nothing is torn.
    unsafe impl BootStorage for Flaky {
        type ReadErr = ();
        type WriteErr = ();

        fn read_raw(&mut self) -> Result<BootStore, ()> {
            Ok(self.held)
        }

        fn write_raw(&mut self, val: BootStore) -> Result<(), ()> {
            if !self.writable {
                return Err(());
            }
            self.held = val;
            Ok(())
        }
    }

    fn flaky() -> Flaky {
        Flaky {
            held: BootStore::default(),
            writable: true,
        }
    }

    /// Records are told apart by their trial count.
    fn counted(n: u8) -> BootStore {
        let mut record = BootStore::default();
        record.boot_state.boot_count = n;
        record
    }

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

    #[test]
    fn a_working_primary_keeps_the_spare_empty() {
        let mut store = Fallback::new(flaky(), flaky());
        store.write(counted(1)).unwrap();

        assert_eq!(store.read().unwrap().boot_state.boot_count, 1);
        assert_eq!(store.primary.held.boot_state.boot_count, 1);
        assert!(
            store.spare.read_raw().unwrap().validated().is_none(),
            "an untouched spare must not look like it holds the record"
        );
    }

    /// The case the whole type exists for: a trial that could not be counted
    /// is counted anyway, so the bootloader still has somewhere to go.
    #[test]
    fn a_failed_primary_write_lands_in_the_spare() {
        let mut store = Fallback::new(flaky(), flaky());
        store.write(counted(1)).unwrap();

        store.primary.writable = false;
        store.write(counted(2)).unwrap();

        assert_eq!(store.read().unwrap().boot_state.boot_count, 2);
        assert_eq!(
            store.primary.held.boot_state.boot_count, 1,
            "the primary is behind, which is why the spare wins"
        );
    }

    #[test]
    fn a_recovered_primary_takes_the_record_back() {
        let mut store = Fallback::new(flaky(), flaky());
        store.primary.writable = false;
        store.write(counted(1)).unwrap();
        assert_eq!(store.read().unwrap().boot_state.boot_count, 1);

        store.primary.writable = true;
        store.write(counted(2)).unwrap();

        assert_eq!(store.primary.held.boot_state.boot_count, 2);
        assert!(
            store.spare.read_raw().unwrap().validated().is_none(),
            "a stale spare would outrank the primary on the next read"
        );
        assert_eq!(store.read().unwrap().boot_state.boot_count, 2);
    }

    /// Losing the spare is losing the writes it was covering for, not losing
    /// the record: the primary is stale but sound.
    #[test]
    fn losing_the_spare_falls_back_to_the_primary() {
        let mut store = Fallback::new(flaky(), flaky());
        store.write(counted(1)).unwrap();
        store.primary.writable = false;
        store.write(counted(2)).unwrap();

        // What backup RAM reads as once power has been away.
        store.spare.held = BootStore::default();

        assert_eq!(store.read().unwrap().boot_state.boot_count, 1);
    }

    /// With neither backend accepting a write there is nothing to report but
    /// failure, and the caller must see it rather than a false success.
    #[test]
    fn both_gone_is_an_error() {
        let mut store = Fallback::new(flaky(), flaky());
        store.primary.writable = false;
        store.spare.writable = false;
        assert!(store.write(counted(1)).is_err());
    }

    /// The codes are a stored format: an application reads them from a
    /// record a bootloader wrote, possibly a different build of one.
    #[test]
    fn codes_are_pinned() {
        assert_eq!(RevertReason::None as u8, 0);
        assert_eq!(RevertReason::AttemptsExhausted as u8, 1);
        assert_eq!(RevertReason::VerifyFailed as u8, 2);
        assert_eq!(RevertReason::AppRejected as u8, 3);
        assert_eq!(BankState::None as u8, 0);
        assert_eq!(BankState::Valid as u8, 1);
        assert_eq!(BankState::New as u8, 2);
        assert_eq!(BankState::Invalid as u8, 3);
    }

    #[test]
    fn a_code_this_build_does_not_know_decodes_to_nothing() {
        for v in 0..=3u8 {
            assert_eq!(RevertReason::from_u8(v).map(|r| r as u8), Some(v));
        }
        assert!(RevertReason::from_u8(4).is_none());
        assert!(RevertReason::from_u8(0xFF).is_none());
    }

    #[test]
    fn the_stored_byte_round_trips() {
        let mut state = BootState::default();
        assert_eq!(state.reason(), Some(RevertReason::None));
        state.set_reason(RevertReason::AppRejected);
        assert_eq!(state.revert_reason, 3);
        assert_eq!(state.reason(), Some(RevertReason::AppRejected));
    }
}

#[cfg(test)]
mod layout_tests {
    use super::*;
    use core::mem::offset_of;

    /// The record is a stored format: a bootloader reads back bytes an older
    /// build of itself wrote, so these offsets are the contract and a change
    /// to any of them is a [`BootStore::FMT`] bump.
    #[test]
    fn record_layout_is_pinned() {
        assert_eq!(size_of::<BootStore>(), 16);
        assert_eq!(offset_of!(BootStore, boot_state), 0);
        assert_eq!(offset_of!(BootStore, rollback), 4);
        assert_eq!(offset_of!(BootStore, mailbox), 8);
        assert_eq!(offset_of!(BootStore, checksum), 12);

        assert_eq!(offset_of!(BootState, states), 0);
        assert_eq!(offset_of!(BootState, boot_count), 1);
        assert_eq!(offset_of!(BootState, revert_reason), 2);
        assert_eq!(offset_of!(BootState, fmt), 3);

        assert_eq!(offset_of!(Rollback, highest_seen_manifest), 0);
        assert_eq!(offset_of!(UpdateMailbox, flags), 0);
    }

    /// The checksum covers everything ahead of it, the format byte included,
    /// so a record cannot claim one layout and be checksummed as another.
    #[test]
    fn the_format_byte_is_under_the_checksum() {
        assert!(offset_of!(BootStore, boot_state) + offset_of!(BootState, fmt) < BootStore::COVERED);
    }

    /// A record from a layout this build does not know is not acted on, and
    /// `read` turns that into the fresh default rather than a misreading.
    #[test]
    fn an_unknown_layout_is_refused() {
        let mut record = BootStore::default();
        record.seal();
        assert!(record.validated().is_some());

        let mut future = record;
        future.boot_state.fmt = BootStore::FMT + 1;
        // Sealed as a valid record of a layout this build cannot read: the
        // checksum agrees and the answer is still no.
        future.checksum = future.computed();
        assert!(future.validated().is_none());
    }

    /// Reserved space is left alone rather than zeroed, so a value a newer
    /// bootloader put there survives a round trip through this one.
    #[test]
    fn reserved_space_survives_a_reader_that_does_not_know_it() {
        let mut record = BootStore::default();
        record.rollback.reserved = 0xBEEF;
        record.mailbox.reserved = [1, 2, 3];
        record.boot_state.states |= 0xF0;
        record.seal();

        let read_back = record.validated().expect("sealed");
        assert_eq!(read_back.rollback.reserved, 0xBEEF);
        assert_eq!(read_back.mailbox.reserved, [1, 2, 3]);
        assert_eq!(read_back.boot_state.states & 0xF0, 0xF0);
    }
}
