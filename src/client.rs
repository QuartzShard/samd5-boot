//! Application-side client for the bootloader's boot record
//!
//! The application talks to BOOT through the same [`BootStorage`] the
//! bootloader keeps its record in: open that store with the same backend and
//! offset BOOT uses, wrap it with [`BootClient::new`], and then, early in
//! `main`:
//!
//! 1. Call [`BootClient::boot_state`] to read why the bootloader last
//!    reverted an image, if it ever has.
//!
//! 1. Call [`BootClient::confirm`] to mark the running image good and take
//!    the trial watchdog over. A trial image that never confirms is reverted
//!    once the attempt budget or the watchdog runs out, so this must land
//!    before the [`trial_timeout`](crate::boot::BootConfig::trial_timeout)
//!    BOOT armed expires.
//!
//! Afterwards [`BootClient::reject`] condemns the running image,
//! [`BootClient::request_update`] asks BOOT to enter download mode on the
//! next boot, and [`BootClient::banks`] reads what the record says about
//! each bank, which is how an application learns whether this boot has
//! anything to fall back on. Every write is a read-modify-write of the boot
//! record.

use atsamd_hal as hal;
use embedded_hal_02::watchdog::{Watchdog as _, WatchdogDisable, WatchdogEnable};
use hal::{
    pac::Wdt,
    watchdog::{Watchdog, WatchdogTimeout},
};

use hal::nvm::{Nvm, PhysicalBank};

use crate::persist::{BankState, BootStorage, RevertReason, UpdateMailbox};

/// What [`BootClient::confirm`] did with the watchdog
pub enum WdtHandoff {
    Reconfigured,
    Disabled,
    /// `Wdt.CTRLA.ALWAYSON` is set: the watchdog can be neither disabled nor
    /// reconfigured, only fed. `cfg` was ignored, and the application must go
    /// on feeding at the period already in force.
    LockedByAlwaysOn,
}

/// Failure of a mailbox update, which reads the record before writing the
/// amended one back
pub enum ClientError<R, W> {
    Read(R),
    Write(W),
}

/// The last rollback the bootloader recorded, for upstream reporting
pub struct BootOutcome {
    /// Why the bootloader last reverted an image, if it ever has. The code
    /// is sticky: [`revert`](crate::boot::Boot::revert) writes it and only a
    /// promotion clears it, so a steady boot long after a rollback still
    /// reports the same reason. `None` is also what an unrecognised code
    /// reads as, which is what a record written by a newer BOOT looks like.
    pub revert_reason: Option<RevertReason>,
}

/// What the boot record says about each physical bank
pub struct Banks {
    /// The bank this image is running from.
    pub active: BankState,
    /// The other bank. [`BankState::Valid`] there is an image
    /// [`fall_back`](crate::boot::Boot::fall_back) would swap to without
    /// verifying it first, so it is also the answer to whether this boot has
    /// anything behind it; anything else means it does not.
    pub inactive: BankState,
}

/// The application-side handle over the shared [`BootStorage`]: the
/// confirm/reject/update mailbox and the last boot's outcome
pub struct BootClient<St> {
    store: St,
}

impl<St: BootStorage> BootClient<St> {
    /// Wrap a store the application already opened
    ///
    /// It must be the same backend, at the same offset, the bootloader keeps
    /// its record in, or the two do not see each other's writes.
    pub fn new(store: St) -> Self {
        Self { store }
    }

    /// Recover the wrapped store
    pub fn free(self) -> St {
        self.store
    }

    fn set_flag(
        &mut self,
        f: impl FnOnce(&mut UpdateMailbox),
    ) -> Result<(), ClientError<St::ReadErr, St::WriteErr>> {
        let mut record = self.store.read().map_err(ClientError::Read)?;
        f(&mut record.mailbox);
        self.store.write(record).map_err(ClientError::Write)
    }

    /// Read what the bootloader recorded on the way to this boot
    pub fn boot_state(&mut self) -> Result<BootOutcome, St::ReadErr> {
        let record = self.store.read()?;
        Ok(BootOutcome {
            revert_reason: record.boot_state.reason(),
        })
    }

    /// Read what the bootloader's record says about each bank
    ///
    /// `nvm` is read only, to name which bank is running.
    pub fn banks(&mut self, nvm: &Nvm) -> Result<Banks, St::ReadErr> {
        let active = nvm.first_bank();
        let inactive = match active {
            PhysicalBank::A => PhysicalBank::B,
            PhysicalBank::B => PhysicalBank::A,
        };
        let record = self.store.read()?;
        Ok(Banks {
            active: record.boot_state.bank(&active),
            inactive: record.boot_state.bank(&inactive),
        })
    }

    /// Mark the running image good and take the trial watchdog over
    ///
    /// Call it early in init. The bootloader promotes the image on the next
    /// boot. The watchdog is fed first and the confirm flag written before
    /// it is reconfigured, so a fault during the changeover is still covered
    /// by a freshly fed watchdog; `cfg` is then applied, or the watchdog
    /// disabled when it is `None`. See [`WdtHandoff`] for the ALWAYSON case.
    ///
    /// On an error the watchdog is left as the bootloader armed it, fed but
    /// still on the trial period, and the caller must go on feeding it.
    pub fn confirm(
        &mut self,
        wdt: &mut Watchdog,
        cfg: Option<WatchdogTimeout>,
    ) -> Result<WdtHandoff, ClientError<St::ReadErr, St::WriteErr>> {
        wdt.feed();

        self.set_flag(|mailbox| mailbox.set_confirmed(true))?;

        // SAFETY: read-only status access
        let always_on = unsafe { &*Wdt::ptr() }.ctrla().read().alwayson().bit_is_set();
        Ok(if always_on {
            WdtHandoff::LockedByAlwaysOn
        } else if let Some(period) = cfg {
            wdt.disable();
            wdt.start(period as u8);
            WdtHandoff::Reconfigured
        } else {
            wdt.disable();
            WdtHandoff::Disabled
        })
    }

    /// Condemn the running image. The bootloader reverts on the next
    /// boot. Safe to call from a panic handler as a fast path to revert;
    /// if the write cannot land, the trial watchdog and attempt counter
    /// remain the backstop.
    pub fn reject(&mut self) -> Result<(), ClientError<St::ReadErr, St::WriteErr>> {
        self.set_flag(|mailbox| mailbox.set_rejected(true))
    }

    /// Ask the bootloader to enter download mode on the next boot
    pub fn request_update(&mut self) -> Result<(), ClientError<St::ReadErr, St::WriteErr>> {
        self.set_flag(|mailbox| mailbox.set_request_update(true))
    }
}
