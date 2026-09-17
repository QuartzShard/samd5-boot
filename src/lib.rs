//! samd5-boot
//!
//! This crate leverages the dual-bank layout of ATSAMD/E5x chips, and the BKSWRST instruction which
//! re-maps them and reboots to implement a safe-update, trial and rollback capable bootloader for
//! these chips.
//!
//! # Memory Layout
//!
#![no_std]
#![no_main]

use core::marker::PhantomData;

use atsamd_hal as hal;

pub mod client;
pub mod consts;
mod flash_writer;
pub mod manifest;
pub mod persist;

pub use flash_writer::FlashError;

use embedded_hal_02::watchdog::WatchdogEnable;
use hal::nvm::PhysicalBank;
use hal::watchdog::{Watchdog, WatchdogTimeout};

use crate::{
    consts::{
        BANK_SIZE, BOOT_SIZE, BOOTPROT_VALUE, FLASH_ADDR, INACTIVE_SLOT_ADDR, MANIFEST_OFFSET,
    },
    manifest::AppManifest,
    persist::{BankState, BootStorage, BootStore, reason},
};

/// Live SEESTAT, not the fuse: the reserve the hardware is operating
/// with right now (DS §25.6.7: 2×SBLK×8 KiB kept clear in each bank).
fn see_reserve(nvm: &hal::nvm::Nvm) -> usize {
    // SAFETY: read-only register access
    let regs = unsafe { nvm.registers() };
    2 * regs.seestat().read().sblk().bits() as usize * 8192
}

pub struct Boot<S: SlotState> {
    nvm: hal::nvm::Nvm,
    dsu: hal::dsu::Dsu,
    wdt: Watchdog,
    config: BootConfig,
    _state: PhantomData<S>,
}

mod seal {
    pub trait Sealed {}
}

/// Track whether the Active/Inactive Slots have been verified This Boot
pub trait SlotState: seal::Sealed {}
pub struct Unverified {}
impl seal::Sealed for Unverified {}
impl SlotState for Unverified {}
pub struct Verified {}
impl seal::Sealed for Verified {}
impl SlotState for Verified {}

pub struct BootConfig {
    /// Trial boots of one image before it is reverted
    pub max_boot_attempts: u8,
    /// Watchdog period armed for trial boots only (1024 Hz clock)
    pub trial_timeout: WatchdogTimeout,
}

pub enum BootConfigError {
    BootprotMisconfigured,
    SmartEEPROMTooLarge,
    /// SBLK 11..=15: reserved encodings; BKSWRST skips SEE reallocation
    /// and no reserve size can be derived from them.
    SmartEEPROMReservedSblk,
}

pub enum VerifyError {
    BadMagic,
    VersionMismatch,
    BadCrc,
    BadLen,
    Dsu(hal::dsu::Error),
}

impl From<hal::dsu::Error> for VerifyError {
    fn from(value: hal::dsu::Error) -> Self {
        Self::Dsu(value)
    }
}

pub enum InstallError<W> {
    Flash(FlashError),
    Verify(VerifyError),
    Write(W),
}

/// What the stored boot record says this boot should do. Read with
/// [`Boot::disposition`]; the caller runs the action each variant calls
/// for, so its own validation or bookkeeping can sit in each arm.
pub enum Disposition {
    SteadyBoot,
    /// `exhausted`: the count has reached `max_boot_attempts`; revert
    /// rather than trying again.
    Trial {
        attempt: u8,
        exhausted: bool,
    },
    /// The trial image reported itself good; promote it and boot.
    Promote,
    /// The application condemned the active image; revert it.
    Reject,
    /// An install lost power before its swap.
    ResumeInstall,
    /// Active image condemned; the other slot holds a valid one.
    Rollback,
    UpdateRequested,
    NoImage,
}

impl<S: SlotState> Boot<S> {
    const fn app_begin(&self, base: usize) -> usize {
        base + BOOT_SIZE
    }

    /// Streams an image's bytes (file order) into the inactive slot's
    /// app region, erasing ahead block by block; bounded below the
    /// slot's SEE reserve, then read-back verified.
    pub fn download(
        &mut self,
        source: impl core::iter::Iterator<Item = u8>,
    ) -> Result<Result<(), VerifyError>, FlashError> {
        let begin = self.app_begin(INACTIVE_SLOT_ADDR);
        let end = INACTIVE_SLOT_ADDR + BANK_SIZE - see_reserve(&self.nvm);
        // SAFETY: `begin` is block-aligned (BOOT_SIZE is a block multiple)
        // and `begin..end` is the inactive slot's app region, and nothing
        // executes there, and the mirror BOOT below `begin` is untouched.
        let writer = unsafe { flash_writer::FlashWriter::new(&mut self.nvm, begin, end) };
        writer.write(flash_writer::pages(flash_writer::words(source)))?;
        Ok(self.check_slot(INACTIVE_SLOT_ADDR))
    }

    /// Swap which bank occupies each slot and reboot: the mirror BOOT
    /// runs from the other bank to boot the newly installed image
    pub fn swap_reboot(mut self) -> ! {
        unsafe { self.nvm.bank_swap() }
    }

    fn inactive_bank(&self) -> PhysicalBank {
        match self.nvm.first_bank() {
            PhysicalBank::A => PhysicalBank::B,
            PhysicalBank::B => PhysicalBank::A,
        }
    }

    /// Download an image into the inactive slot, mark it for a trial
    /// boot, and swap the banks (which reboots). Returns only on failure.
    pub fn install<St: BootStorage>(
        mut self,
        store: &mut St,
        mut record: BootStore,
        source: impl core::iter::Iterator<Item = u8>,
    ) -> (InstallError<St::WriteErr>, Self) {
        match self.download(source) {
            Err(e) => return (InstallError::Flash(e), self),
            Ok(Err(e)) => return (InstallError::Verify(e), self),
            Ok(Ok(())) => (),
        }
        record.boot_state.mark_new(&self.inactive_bank());
        record.boot_state.boot_count = 0;
        if let Err(e) = store.write(record) {
            return (InstallError::Write(e), self);
        }
        self.swap_reboot()
    }

    /// Abandon the active slot's image: mark it `Invalid` with `reason`,
    /// then swap back to the other bank (which reboots). Returns only on
    /// failure.
    pub fn revert<St: BootStorage>(
        self,
        store: &mut St,
        mut record: BootStore,
        reason: u8,
    ) -> (St::WriteErr, Self) {
        record
            .boot_state
            .set_bank(&self.nvm.first_bank(), BankState::Invalid);
        record.boot_state.revert_reason = reason;
        if let Err(e) = store.write(record) {
            return (e, self);
        }
        self.swap_reboot()
    }

    /// Verify the integrity of the slot starting at `base`
    fn check_slot(&mut self, base: usize) -> Result<(), VerifyError> {
        let manifest_addr = (self.app_begin(base) + MANIFEST_OFFSET) as *const AppManifest;
        let manifest: AppManifest = unsafe { manifest_addr.read_volatile() };
        if manifest.body.magic != manifest::MAGIC {
            return Err(VerifyError::BadMagic);
        }
        // Image must contain at least the fields we expect. Extras are ignored
        if manifest.body.fmt_version < manifest::FMT_VER {
            return Err(VerifyError::VersionMismatch);
        }
        let len = manifest.body.image_len as usize;
        if !(MANIFEST_OFFSET + size_of::<AppManifest>()
            ..=BANK_SIZE - BOOT_SIZE - see_reserve(&self.nvm))
            .contains(&len)
            || !len.is_multiple_of(4)
        {
            return Err(VerifyError::BadLen);
        }
        // Errata 2.7.1 (CHIP003-171): the DSU CRC32 never completes while
        // the NVM cache is disabled. Force it on (also the desired steady
        // state); `modify` leaves RWS/WMODE untouched.
        unsafe {
            self.nvm.registers().ctrla().modify(|_, w| {
                w.cachedis0().clear_bit();
                w.cachedis1().clear_bit()
            });
        }
        // Coverage splits around the two crc fields at the manifest head:
        // everything else in the image, the rest of the manifest
        // included, sits under one of the two CRCs.
        let body_offset = MANIFEST_OFFSET + core::mem::offset_of!(AppManifest, body);
        let ranges = [
            (
                self.app_begin(base),
                MANIFEST_OFFSET,
                manifest.head.crc32_vec_table,
            ),
            (
                self.app_begin(base) + body_offset,
                len - body_offset,
                manifest.head.crc32_image,
            ),
        ];
        for (addr, range_len, expected) in ranges {
            match self.dsu.crc32(addr as u32, range_len as u32) {
                Ok(crc) if crc != expected => return Err(VerifyError::BadCrc),
                Err(e) => return Err(VerifyError::from(e)),
                _ => (),
            }
        }

        Ok(())
    }

    /// Dissolve the bootloader, handing the peripherals back
    pub fn free(self) -> (hal::nvm::Nvm, hal::dsu::Dsu, Watchdog, BootConfig) {
        (self.nvm, self.dsu, self.wdt, self.config)
    }
}

impl Boot<Unverified> {
    /// Construct the bootloader.
    /// Returns error when preconditions not met. (misconfigured bootprot, SmartEEPROM, etc)
    pub fn new(
        nvm: hal::nvm::Nvm,
        dsu: hal::dsu::Dsu,
        wdt: Watchdog,
        config: BootConfig,
    ) -> Result<Self, BootConfigError> {
        let userpage = nvm.read_userpage();
        if userpage.nvm_bootloader_size() != BOOTPROT_VALUE {
            return Err(BootConfigError::BootprotMisconfigured);
        }
        // SAFETY: read-only register access
        if unsafe { nvm.registers() }.seestat().read().sblk().bits() > 10 {
            return Err(BootConfigError::SmartEEPROMReservedSblk);
        }
        if BOOT_SIZE + see_reserve(&nvm) >= BANK_SIZE {
            return Err(BootConfigError::SmartEEPROMTooLarge);
        }

        Ok(Boot {
            nvm,
            dsu,
            wdt,
            config,
            _state: PhantomData,
        })
    }

    /// Infallibly construct the bootloader, configuring any fuses required.
    /// For use in "provisioning" call paths that assume blank-slate state.
    pub fn init(nvm: hal::nvm::Nvm, dsu: hal::dsu::Dsu, wdt: Watchdog, config: BootConfig) -> Self {
        // Maybe this needs to reboot rather than returning self
        todo!();
        Boot {
            nvm,
            dsu,
            wdt,
            config,
            _state: PhantomData,
        }
    }

    /// Read the stored boot record and classify what this boot should
    /// do, without acting on it. Hands back the record it read so the
    /// caller can act without reading it a second time.
    pub fn disposition<St: BootStorage>(
        &self,
        store: &mut St,
    ) -> Result<(Disposition, BootStore), St::ReadErr> {
        let record = store.read()?;
        let active = record.boot_state.bank(&self.nvm.first_bank());
        let inactive = record.boot_state.bank(&self.inactive_bank());
        let mailbox = record.mailbox;
        // Precedence: finishing an interrupted install overrides stale
        // flags about the image it replaces; an explicit reject or
        // confirm about the running image comes before a download
        // request; then the active image's own state decides.
        let disposition = if inactive == BankState::New {
            Disposition::ResumeInstall
        } else if mailbox.rejected() {
            Disposition::Reject
        } else if active == BankState::New && mailbox.confirmed() {
            Disposition::Promote
        } else if mailbox.request_update() {
            Disposition::UpdateRequested
        } else {
            match active {
                BankState::New => {
                    let attempt = record.boot_state.boot_count.saturating_add(1);
                    Disposition::Trial {
                        attempt,
                        exhausted: attempt > self.config.max_boot_attempts,
                    }
                }
                BankState::Valid | BankState::None => Disposition::SteadyBoot,
                BankState::Invalid if inactive == BankState::Valid => Disposition::Rollback,
                BankState::Invalid => Disposition::NoImage,
            }
        };
        Ok((disposition, record))
    }

    /// Default handling of [`Boot::disposition`]: boots the image (or
    /// swaps banks and reboots) on every bootable outcome, and returns
    /// `self` for the caller to enter download mode when an update was
    /// requested or nothing is bootable. Write your own match over
    /// `disposition` instead to add your own validation or bookkeeping.
    pub fn boot_or_enter_download<St: BootStorage>(self, store: &mut St) -> Self {
        let (disposition, mut record) = self
            .disposition(store)
            .unwrap_or((Disposition::SteadyBoot, BootStore::default()));
        match disposition {
            Disposition::UpdateRequested => {
                record.mailbox.set_request_update(false);
                let _ = store.write(record);
                self
            }
            Disposition::ResumeInstall | Disposition::Rollback => self.swap_reboot(),
            Disposition::NoImage => self,
            Disposition::Reject => {
                // Clear the flag before reverting: an uncleared reject
                // would condemn the bank we swap into as well.
                record.mailbox.set_rejected(false);
                if store.write(record).is_err() {
                    return self;
                }
                self.revert(store, record, reason::APP_REJECTED).1
            }
            Disposition::Promote => {
                record
                    .boot_state
                    .set_bank(&self.nvm.first_bank(), BankState::Valid);
                record.boot_state.boot_count = 0;
                record.mailbox.set_confirmed(false);
                if store.write(record).is_err() {
                    return self;
                }
                match self.verify() {
                    Ok(verified) => verified.boot_active(),
                    Err((_, boot)) => boot,
                }
            }
            Disposition::SteadyBoot => match self.verify() {
                Ok(verified) => verified.boot_active(),
                Err((_, boot)) => {
                    if record.boot_state.bank(&boot.inactive_bank()) == BankState::Valid {
                        boot.swap_reboot()
                    }
                    boot
                }
            },
            Disposition::Trial { exhausted, .. } => {
                if exhausted {
                    return self.revert(store, record, reason::ATTEMPTS_EXHAUSTED).1;
                }
                match self.verify() {
                    // A store-write failure means the trial can't be
                    // counted; drop to download rather than boot uncounted.
                    Ok(verified) => verified.boot_trial(store, record).1.denounce(),
                    Err((_, boot)) => boot.revert(store, record, reason::VERIFY_FAILED).1,
                }
            }
        }
    }

    /// Verify integrity of the active slot, unlocking boot
    pub fn verify(mut self) -> Result<Boot<Verified>, (VerifyError, Boot<Unverified>)> {
        match self.check_slot(FLASH_ADDR) {
            Ok(_) => Ok(Boot {
                nvm: self.nvm,
                dsu: self.dsu,
                wdt: self.wdt,
                config: self.config,
                _state: PhantomData,
            }),
            Err(e) => Err((e, self)),
        }
    }
}

impl Boot<Verified> {
    /// Persist the incremented trial count, arm the watchdog, and jump.
    /// Returns only if the store write fails. The count must land
    /// before the jump, or a crash loop never gets counted.
    pub fn boot_trial<St: BootStorage>(
        mut self,
        store: &mut St,
        mut record: BootStore,
    ) -> (St::WriteErr, Self) {
        record.boot_state.boot_count = record.boot_state.boot_count.saturating_add(1);
        if let Err(e) = store.write(record) {
            return (e, self);
        }
        self.wdt.start(self.config.trial_timeout as u8);
        self.boot_active()
    }

    /// Drop the verified proof, for a caller that decides against
    /// booting after `verify` (e.g. its own check failed).
    pub fn denounce(self) -> Boot<Unverified> {
        Boot {
            nvm: self.nvm,
            dsu: self.dsu,
            wdt: self.wdt,
            config: self.config,
            _state: PhantomData,
        }
    }

    /// Jump to the program
    pub fn boot_active(self) -> ! {
        let application_addr = self.app_begin(FLASH_ADDR);
        // Geronimo
        unsafe {
            let scb = &*cortex_m::peripheral::SCB::PTR;
            scb.vtor.write(application_addr as u32);
            cortex_m::asm::bootload(application_addr as *const _)
        }
    }
}
