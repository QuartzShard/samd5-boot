//! Application-side client. What the app running on top of a
//! samd5-boot image calls to confirm a trial, condemn itself, request an
//! update, or read the previous boot's outcome. Built over the same
//! [`BootStorage`] the bootloader uses, so the application constructs a
//! [`SmartEepromStore`](crate::persist::SmartEepromStore) at the same
//! offset and hands it here.

use atsamd_hal as hal;
use embedded_hal_02::watchdog::{Watchdog as _, WatchdogDisable, WatchdogEnable};
use hal::{
    pac::Wdt,
    watchdog::{Watchdog, WatchdogTimeout},
};

use crate::persist::{BootStorage, UpdateMailbox};

/// What [`BootClient::confirm`] did with the watchdog.
pub enum WdtHandoff {
    Reconfigured,
    Disabled,
    /// `CTRLA.ALWAYSON` is fused: the watchdog can be neither disabled
    /// nor reconfigured, only fed. The application is stuck with the
    /// period BOOT armed.
    LockedByAlwaysOn,
}

/// The previous boot's outcome, for upstream reporting.
pub struct BootOutcome {
    /// Nonzero if the bootloader reverted an image on the way here; the
    /// value is a [`reason`](crate::persist::reason) code.
    pub revert_reason: u8,
}

/// The application-side handle over the shared [`BootStorage`]: the
/// confirm/reject/update mailbox and the last boot's outcome.
pub struct BootClient<St> {
    store: St,
}

impl<St: BootStorage> BootClient<St> {
    /// Wrap a store the application already opened (at the bootloader's
    /// offset).
    pub fn new(store: St) -> Self {
        Self { store }
    }

    /// Recover the wrapped store.
    pub fn free(self) -> St {
        self.store
    }

    fn set_flag(&mut self, f: impl FnOnce(&mut UpdateMailbox)) -> Result<(), St::WriteErr> {
        let mut record = self.store.read().unwrap_or_default();
        f(&mut record.mailbox);
        self.store.write(record)
    }

    /// Read what the bootloader recorded on the way to this boot.
    pub fn boot_state(&mut self) -> Result<BootOutcome, St::ReadErr> {
        let record = self.store.read()?;
        Ok(BootOutcome {
            revert_reason: record.boot_state.revert_reason,
        })
    }

    /// Call early in init. Marks the running image good, so the
    /// bootloader promotes it on the next boot, and hands the trial
    /// watchdog to the application: feeds it, then applies `cfg` (or
    /// disables it when `None`). Writing the confirm flag comes before
    /// the watchdog is touched, so a fault during the changeover still
    /// has the fed watchdog covering it. See [`WdtHandoff`] for the
    /// ALWAYSON case.
    pub fn confirm(
        &mut self,
        wdt: &mut Watchdog,
        cfg: Option<WatchdogTimeout>,
    ) -> Result<WdtHandoff, St::WriteErr> {
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
    pub fn reject(&mut self) -> Result<(), St::WriteErr> {
        self.set_flag(|mailbox| mailbox.set_rejected(true))
    }

    /// Ask the bootloader to enter download mode on the next boot.
    pub fn request_update(&mut self) -> Result<(), St::WriteErr> {
        self.set_flag(|mailbox| mailbox.set_request_update(true))
    }
}
