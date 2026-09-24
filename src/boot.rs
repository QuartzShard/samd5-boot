//! The bootloader state machine
//!
//! A BOOT binary uses this in four steps. First, build a [`Boot`] from the
//! NVM, DSU and watchdog with [`Boot::new`], which checks the fuses it will
//! boot under. Next, either hand the decision to
//! [`Boot::boot_or_enter_download`] or read [`Boot::disposition`] and match
//! the [`Disposition`] yourself. Either way, a jump into the active image
//! goes through [`Boot::verify`]: its `Boot<Verified>` is the only thing
//! that unlocks [`boot_active`](Boot::<Verified>::boot_active). Finally, a
//! call that returns instead of booting leaves the binary in download mode,
//! where it drives its own transport into [`Boot::install`].
//!
//! [`Boot::revert`] abandons the active image and swaps back;
//! [`Boot::free`] hands the peripherals out again.
//!
//! Every item here drives NVMCTRL, the DSU or the watchdog, so the whole
//! module sits behind the `target` feature; a host build of this crate keeps
//! the flash ABI and drops all of it.

use core::marker::PhantomData;

use atsamd_hal as hal;

use embedded_hal_02::watchdog::WatchdogEnable;
use hal::nvm::PhysicalBank;
use hal::watchdog::{Watchdog, WatchdogTimeout};

use crate::{
    consts::{
        ACTIVE_SLOT_ADDR, BANK_SIZE, BOOT_REGIONS, BOOT_SIZE, BOOTPROT_VALUE, INACTIVE_SLOT_ADDR,
        MANIFEST_OFFSET,
    },
    flash_writer::{self, FlashError},
    manifest::{self, AppManifest},
    persist::{BankState, BootStorage, BootStore, RevertReason},
};

/// Live SEESTAT, not the fuse: the reserve the hardware is operating with
/// right now. The SmartEEPROM keeps 2 × SBLK blocks of 8 KiB clear in each
/// bank.
fn see_reserve(nvm: &hal::nvm::Nvm) -> usize {
    // SAFETY: read-only register access
    let regs = unsafe { nvm.registers() };
    2 * regs.seestat().read().sblk().bits() as usize * 8192
}

/// The bootloader state machine, owning the peripherals it drives (NVM,
/// DSU, watchdog) and the [`BootConfig`] policy. `S` is a [`SlotState`]
/// typestate that records whether the active slot has been verified this
/// boot: only [`Boot::verify`] produces a [`Boot<Verified>`], and only that
/// unlocks [`boot_active`](Boot::<Verified>::boot_active). Construct with
/// [`Boot::new`]; hand the peripherals back with
/// [`Boot::free`].
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

/// Sealed typestate marking whether [`Boot`]'s active slot has been verified
/// this boot: either [`Unverified`] or [`Verified`].
pub trait SlotState: seal::Sealed {}

/// The initial state of a freshly constructed [`Boot`].
pub struct Unverified {}
impl seal::Sealed for Unverified {}
impl SlotState for Unverified {}

/// Reached only through [`Boot::verify`].
pub struct Verified {}
impl seal::Sealed for Verified {}
impl SlotState for Verified {}

/// Downstream-set boot policy.
pub struct BootConfig {
    /// Trial boots of one image before it is reverted
    pub max_boot_attempts: u8,
    /// Watchdog period armed for trial boots only (1024 Hz clock)
    pub trial_timeout: WatchdogTimeout,
}

/// A precondition [`Boot::new`] found unmet: the live fuse configuration is
/// incompatible with a swap-safe layout. All three are provisioning faults,
/// fixed off-board with `samd5-boot-tools provision`; each variant's
/// condition is listed under [`Boot::new`]'s `# Errors`.
pub enum BootConfigError {
    BootprotMisconfigured,
    SmartEEPROMTooLarge,
    SmartEEPROMReservedSblk,
}

/// Why an image failed integrity verification: the first check to fail.
/// The checks run cheapest first, in the order `BadMagic`,
/// `VersionMismatch`, `BadLen`, `BadCrc`, which is not the order the
/// variants are declared in.
pub enum VerifyError {
    BadMagic,
    VersionMismatch,
    BadCrc,
    BadLen,
    /// The DSU could not run the CRC at all (unaligned range, PAC unlock,
    /// bus error), as opposed to a CRC that came back wrong.
    Dsu(hal::dsu::Error),
}

impl From<hal::dsu::Error> for VerifyError {
    fn from(value: hal::dsu::Error) -> Self {
        Self::Dsu(value)
    }
}

/// Failure of the composed [`Boot::install`] verb. `Write` is the trial
/// record, not the image; `W` is the store's write-error type.
pub enum InstallError<W> {
    Flash(FlashError),
    Verify(VerifyError),
    Write(W),
}

/// Failure of [`Boot::download`].
pub enum DownloadError {
    Flash(FlashError),
    Verify(VerifyError),
}

impl<W> From<DownloadError> for InstallError<W> {
    fn from(e: DownloadError) -> Self {
        match e {
            DownloadError::Flash(e) => Self::Flash(e),
            DownloadError::Verify(e) => Self::Verify(e),
        }
    }
}

/// A verb that could not complete, handing the [`Boot`] it consumed back so
/// the caller can take another route with the peripherals. Dropping one
/// drops the NVM, DSU and watchdog with it, which is why it is `must_use`.
#[must_use]
pub struct Aborted<E, B> {
    pub error: E,
    pub boot: B,
}

/// What the stored boot record says this boot should do. Read with
/// [`Boot::disposition`]; the caller runs the action each variant calls
/// for, so its own validation or bookkeeping can sit in each arm.
pub enum Disposition {
    SteadyBoot,
    /// `attempt` is this boot's 1-based trial count.
    Trial { attempt: u8 },
    /// The trial budget is spent: revert rather than boot again. `attempt`
    /// is this boot's 1-based count.
    TrialExhausted { attempt: u8 },
    Promote,
    Reject,
    /// An install recorded its trial but the swap never happened. Swapping
    /// now finishes it; the image was read-back verified when it was
    /// written.
    ResumeInstall,
    /// The active image is condemned and the other bank holds a confirmed
    /// one. Swapping is the only action needed: the record already says so.
    Rollback,
    UpdateRequested,
    /// The active image is condemned and no bank holds a confirmed one:
    /// nothing on the part is known to boot.
    NoImage,
}

const fn app_begin(base: usize) -> usize {
    base + BOOT_SIZE
}

impl<S: SlotState> Boot<S> {
    fn app_region_len(&self) -> usize {
        BANK_SIZE - BOOT_SIZE - see_reserve(&self.nvm)
    }

    /// Stream an image's bytes (file order) into the inactive slot's app
    /// region, then read it back and verify it
    ///
    /// Erases ahead block by block, so no separate erase pass is needed,
    /// and holds the NVM cache disabled while it writes (errata 2.14.1).
    /// Bounded below the slot's live SmartEEPROM reserve.
    pub fn download(
        &mut self,
        source: impl core::iter::Iterator<Item = u8>,
    ) -> Result<(), DownloadError> {
        let begin = app_begin(INACTIVE_SLOT_ADDR);
        let end = begin + self.app_region_len();
        // SAFETY: `begin` is block-aligned (BOOT_SIZE is a block multiple)
        // and `begin..end` is the inactive slot's app region, and nothing
        // executes there, and the mirror BOOT below `begin` is untouched.
        let writer = unsafe { flash_writer::FlashWriter::new(&mut self.nvm, begin, end) };
        writer
            .write(flash_writer::pages(flash_writer::words(source)))
            .map_err(DownloadError::Flash)?;
        self.check_slot(INACTIVE_SLOT_ADDR)
            .map_err(DownloadError::Verify)
    }

    /// Swap which bank occupies each slot and reboot: the mirror BOOT
    /// runs from the other bank to boot the newly installed image.
    ///
    /// The other bank must already hold a working image. Nothing here
    /// checks it, and the reset lands in whatever sits at its base:
    /// [`Boot::install`] reaches this only after [`Boot::download`]'s
    /// read-back, and [`Boot::fall_back`] only with a bank recorded
    /// [`BankState::Valid`], but [`Boot::revert`] and the `ResumeInstall`
    /// and `Rollback` arms of [`Boot::boot_or_enter_download`] reach it
    /// without checking the destination.
    ///
    /// Interrupts are disabled first and never restored, because the command
    /// ends in a reset. BKSWRST stalls the AHB interfaces and forbids any NVM
    /// fetch while it runs (DS60001507 section 25.6.7), so an interrupt taken
    /// during it sends the core to a vector it cannot fetch. The window is
    /// long enough to matter whenever SmartEEPROM is configured: the command
    /// then also reallocates the SEE sector, erasing and copying before it
    /// resets.
    pub fn swap_reboot(mut self) -> ! {
        cortex_m::interrupt::disable();
        unsafe { self.nvm.bank_swap() }
    }

    fn inactive_bank(&self) -> PhysicalBank {
        match self.nvm.first_bank() {
            PhysicalBank::A => PhysicalBank::B,
            PhysicalBank::B => PhysicalBank::A,
        }
    }

    /// Download an image into the inactive slot, verify it, mark it for a
    /// trial boot, and swap the banks (which reboots)
    ///
    /// Returns only on failure, handing the [`Boot`] back in an
    /// [`Aborted`].
    ///
    /// Clears the confirm and reject flags as it records the trial: both
    /// refer to the image being replaced, and carrying one over would let
    /// the outgoing image's confirmation promote the incoming one before it
    /// has served a trial at all.
    pub fn install<St: BootStorage>(
        mut self,
        store: &mut St,
        mut record: BootStore,
        source: impl core::iter::Iterator<Item = u8>,
    ) -> Aborted<InstallError<St::WriteErr>, Self> {
        if let Err(e) = self.download(source) {
            return Aborted {
                error: e.into(),
                boot: self,
            };
        }
        record.boot_state.mark_new(&self.inactive_bank());
        record.boot_state.boot_count = 0;
        record.mailbox.set_confirmed(false);
        record.mailbox.set_rejected(false);
        if let Err(e) = store.write(record) {
            return Aborted {
                error: InstallError::Write(e),
                boot: self,
            };
        }
        self.swap_reboot()
    }

    /// Hand over to the other bank because the active image did not
    /// verify, when that bank holds a confirmed image to hand over to.
    /// Returns for the caller to drop to download when it does not, which
    /// is the case where nothing on the part is known to boot.
    pub fn fall_back<St: BootStorage>(
        self,
        store: &mut St,
        record: BootStore,
        reason: RevertReason,
    ) -> Self {
        if record.boot_state.bank(&self.inactive_bank()) == BankState::Valid {
            return self.revert(store, record, reason).boot;
        }
        self
    }

    /// Abandon the active slot's image: mark it `Invalid` with `reason`,
    /// then swap back to the other bank (which reboots). Returns only on
    /// failure.
    pub fn revert<St: BootStorage>(
        self,
        store: &mut St,
        mut record: BootStore,
        reason: RevertReason,
    ) -> Aborted<St::WriteErr, Self> {
        record
            .boot_state
            .set_bank(&self.nvm.first_bank(), BankState::Invalid);
        record.boot_state.set_reason(reason);
        if let Err(e) = store.write(record) {
            return Aborted {
                error: e,
                boot: self,
            };
        }
        self.swap_reboot()
    }

    /// Verify the integrity of the slot starting at `base`
    fn check_slot(&mut self, base: usize) -> Result<(), VerifyError> {
        let manifest_addr = (app_begin(base) + MANIFEST_OFFSET) as *const AppManifest;
        let manifest: AppManifest = unsafe { manifest_addr.read_volatile() };
        if manifest.body.magic != manifest::MAGIC {
            return Err(VerifyError::BadMagic);
        }
        // Image must contain at least the fields we expect. Extras are ignored
        if manifest.body.fmt_version < manifest::FMT_VER {
            return Err(VerifyError::VersionMismatch);
        }
        let len = manifest.body.image_len as usize;
        if !(manifest::MIN_IMAGE_LEN..=self.app_region_len()).contains(&len)
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
        let ranges = [
            (
                app_begin(base),
                MANIFEST_OFFSET,
                manifest.head.crc32_vec_table,
            ),
            (
                app_begin(base) + manifest::OFF_BODY,
                len - manifest::OFF_BODY,
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

    /// Lock the flash regions holding both copies of BOOT.
    ///
    /// BOOTPROT covers only the BOOT at the base of the *active* bank, which
    /// leaves the mirror in the inactive bank writable. The power-on
    /// default should come from the user page's region lock bits (written by
    /// `samd5-boot-tools provision`); this re-asserts it at runtime.
    ///
    /// Idempotent and non-clobbering: the current `RUNLOCK` is
    /// read back and only the BOOT bits are cleared, so locks the
    /// application set elsewhere survive.
    pub fn lock_boot_regions(&mut self) -> Result<(), hal::nvm::Error> {
        // SAFETY: read-only register access.
        let runlock = unsafe { self.nvm.registers() }.runlock().read().bits();
        // A clear bit locks the region.
        self.nvm.region_lock(runlock & !BOOT_REGIONS)
    }

    /// Dissolve the bootloader, handing the peripherals back
    pub fn free(self) -> (hal::nvm::Nvm, hal::dsu::Dsu, Watchdog, BootConfig) {
        (self.nvm, self.dsu, self.wdt, self.config)
    }
}

impl Boot<Unverified> {
    /// Construct the bootloader, validating the fuse configuration it will
    /// boot under
    ///
    /// Also re-asserts the BOOT region locks with
    /// [`Boot::lock_boot_regions`]. A part that will not take them is not
    /// refused: the image is intact either way, and BOOTPROT still covers
    /// the active copy.
    ///
    /// # Errors
    ///
    /// * Returns [`BootConfigError::BootprotMisconfigured`] if the user
    ///   page's BOOTPROT field is not [`BOOTPROT_VALUE`], so BOOT is
    ///   protected to the wrong size or not at all.
    /// * Returns [`BootConfigError::SmartEEPROMReservedSblk`] if
    ///   `Nvmctrl.SEESTAT.SBLK` is above 10, a reserved encoding.
    /// * Returns [`BootConfigError::SmartEEPROMTooLarge`] if the
    ///   SmartEEPROM reserve and the BOOT region together leave no
    ///   application region in a bank.
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

        let mut boot = Boot {
            nvm,
            dsu,
            wdt,
            config,
            _state: PhantomData,
        };
        // A part whose locks cannot be driven is not a reason to refuse to
        // boot: the image is still intact, and BOOTPROT still covers the
        // active copy.
        let _ = boot.lock_boot_regions();
        Ok(boot)
    }

    /// Read the stored boot record and classify what this boot should do,
    /// without acting on it
    ///
    /// Hands back the record it read so the caller can act without reading
    /// it a second time.
    ///
    /// The checks run in this order and the first match wins:
    ///
    /// 1. The inactive bank is [`BankState::New`]: an install recorded its
    ///    trial but never swapped, so [`Disposition::ResumeInstall`].
    /// 1. [`rejected`](crate::persist::UpdateMailbox::rejected):
    ///    [`Disposition::Reject`].
    /// 1. The active bank is `New` and
    ///    [`confirmed`](crate::persist::UpdateMailbox::confirmed):
    ///    [`Disposition::Promote`]. A confirm about an image that is not on
    ///    trial is left alone.
    /// 1. [`request_update`](crate::persist::UpdateMailbox::request_update):
    ///    [`Disposition::UpdateRequested`].
    /// 1. Otherwise the active bank's own [`BankState`] decides.
    pub fn disposition<St: BootStorage>(
        &self,
        store: &mut St,
    ) -> Result<(Disposition, BootStore), St::ReadErr> {
        let record = store.read()?;
        let active = record.boot_state.bank(&self.nvm.first_bank());
        let inactive = record.boot_state.bank(&self.inactive_bank());
        let mailbox = record.mailbox;
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
                    if attempt > self.config.max_boot_attempts {
                        Disposition::TrialExhausted { attempt }
                    } else {
                        Disposition::Trial { attempt }
                    }
                }
                BankState::Valid | BankState::None => Disposition::SteadyBoot,
                BankState::Invalid if inactive == BankState::Valid => Disposition::Rollback,
                BankState::Invalid => Disposition::NoImage,
            }
        };
        Ok((disposition, record))
    }

    /// Default handling of [`Boot::disposition`]: boot the image, or swap
    /// banks and reboot, on every bootable outcome
    ///
    /// Returns `self` for the caller to enter download mode in three
    /// cases: the application asked for an update, nothing on the part is
    /// known to boot, or a store write failed, which leaves no way to
    /// record the promotion, trial or revert this boot called for. A store
    /// that cannot be *read* is not one of them: the record is taken to be
    /// the fresh default and the boot proceeds as
    /// [`Disposition::SteadyBoot`], so the active image is still verified
    /// before the jump.
    ///
    /// Match over [`Disposition`] yourself instead to add your own
    /// validation or bookkeeping.
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
                self.revert(store, record, RevertReason::AppRejected).boot
            }
            // Verified before the promotion is recorded, so an image that
            // cannot be booted is never written down as good.
            Disposition::Promote => match self.verify() {
                Ok(verified) => {
                    record
                        .boot_state
                        .set_bank(&verified.nvm.first_bank(), BankState::Valid);
                    record.boot_state.boot_count = 0;
                    // A trial that succeeded supersedes whatever rollback
                    // came before it, so the reason has served its purpose.
                    record.boot_state.set_reason(RevertReason::None);
                    record.mailbox.set_confirmed(false);
                    verified.boot_recorded(store, record).boot.denounce()
                }
                Err(Aborted { boot, .. }) => {
                    boot.fall_back(store, record, RevertReason::VerifyFailed)
                }
            },
            Disposition::SteadyBoot => match self.verify() {
                Ok(verified) => verified.boot_active(),
                Err(Aborted { boot, .. }) => {
                    boot.fall_back(store, record, RevertReason::VerifyFailed)
                }
            },
            Disposition::TrialExhausted { .. } => {
                self.revert(store, record, RevertReason::AttemptsExhausted).boot
            }
            Disposition::Trial { .. } => match self.verify() {
                // Dropping to download is the terminal state for a store
                // that cannot be written: swapping away instead would
                // leave this bank inactive and still `New`, which the next
                // boot reads as an interrupted install and swaps back.
                Ok(verified) => verified.boot_trial(store, record).boot.denounce(),
                // Condemn rather than `fall_back`: with nothing bootable
                // in the other bank `fall_back` returns here, leaving this
                // bank `New` with its count unmoved (only `boot_trial`
                // advances it), so every later reset re-runs the same
                // failing trial.
                Err(Aborted { boot, .. }) => {
                    boot.revert(store, record, RevertReason::VerifyFailed).boot
                }
            },
        }
    }

    /// Verify integrity of the active slot, unlocking boot
    pub fn verify(mut self) -> Result<Boot<Verified>, Aborted<VerifyError, Boot<Unverified>>> {
        match self.check_slot(ACTIVE_SLOT_ADDR) {
            Ok(_) => Ok(Boot {
                nvm: self.nvm,
                dsu: self.dsu,
                wdt: self.wdt,
                config: self.config,
                _state: PhantomData,
            }),
            Err(e) => Err(Aborted {
                error: e,
                boot: self,
            }),
        }
    }
}

impl Boot<Verified> {
    /// Commit `record`, then jump. Returns only if the write failed, with
    /// the proof intact so the caller can discard it: booting an image
    /// whose bookkeeping was lost leaves a trial that can never resolve,
    /// and dropping to download is what the bootloader has a transport for.
    pub fn boot_recorded<St: BootStorage>(
        self,
        store: &mut St,
        record: BootStore,
    ) -> Aborted<St::WriteErr, Self> {
        if let Err(e) = store.write(record) {
            return Aborted {
                error: e,
                boot: self,
            };
        }
        self.boot_active()
    }

    /// Count this trial boot, arm the trial watchdog, then jump
    ///
    /// Increments [`boot_count`](crate::persist::BootState::boot_count) and
    /// commits `record` before starting the watchdog at
    /// [`BootConfig::trial_timeout`], so a crash loop is counted even when
    /// the image never reaches its own code. The application takes the
    /// watchdog over with
    /// [`confirm`](crate::client::BootClient::confirm); an image that does
    /// not is reset until [`BootConfig::max_boot_attempts`] is spent, at
    /// which point [`Boot::disposition`] returns
    /// [`Disposition::TrialExhausted`].
    ///
    /// Returns only if the store write fails.
    pub fn boot_trial<St: BootStorage>(
        mut self,
        store: &mut St,
        mut record: BootStore,
    ) -> Aborted<St::WriteErr, Self> {
        record.boot_state.boot_count = record.boot_state.boot_count.saturating_add(1);
        if let Err(e) = store.write(record) {
            return Aborted {
                error: e,
                boot: self,
            };
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
        let application_addr = app_begin(ACTIVE_SLOT_ADDR);
        // SAFETY: the `Verified` typestate means `check_slot` passed on the
        // active slot, so the SP and reset vector read here are covered by
        // `crc32_vec_table` rather than erased flash.
        unsafe {
            let scb = &*cortex_m::peripheral::SCB::PTR;
            scb.vtor.write(application_addr as u32);
            cortex_m::asm::bootload(application_addr as *const _)
        }
    }
}
