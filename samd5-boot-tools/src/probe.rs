//! One attached, halted debug session, and the NVMCTRL sequences that have
//! to run inside it
//!
//! The NVM page buffer is volatile and auxiliary pages have no
//! read-while-write, so an erase, a buffer fill and the quad-word commit
//! that follows must be one uninterrupted sequence against a core that is
//! not itself driving NVMCTRL.
//!
//! Register offsets and bit positions are DS60001507 section 25.8, as are
//! the `CTRLB.CMD` values in [`Cmd`] that this crate issues. User-page
//! field positions live in [`crate::provision`].

use std::fmt;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use probe_rs::{Core, MemoryInterface, Session, SessionConfig, VectorCatchCondition};

const NVMCTRL: u64 = 0x4100_4000;
const CTRLA: u64 = NVMCTRL;
const CTRLB: u64 = NVMCTRL + 0x04;
const PARAM: u64 = NVMCTRL + 0x08;
const INTFLAG: u64 = NVMCTRL + 0x10;
const STATUS: u64 = NVMCTRL + 0x12;
const ADDR: u64 = NVMCTRL + 0x14;
const RUNLOCK: u64 = NVMCTRL + 0x18;
const SEESTAT: u64 = NVMCTRL + 0x2C;

/// `CTRLA.CACHEDIS0` and `CACHEDIS1`
///
/// Errata NVM101-7 leaves the NVM cache holding stale lines after
/// programming.
const CACHEDIS: u16 = (1 << 14) | (1 << 15);

/// `CTRLB.CMDEX`: a command executes only with this key in bits 15:8
const CMD_KEY: u16 = 0xA500;

/// NVMCTRL commands this crate issues, as `CTRLB.CMD` values
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum Cmd {
    Ep = 0,
    Wqw = 4,
    Pbc = 21,
    Bkswrst = 23,
}

impl Cmd {
    fn name(self) -> &'static str {
        match self {
            Cmd::Ep => "EP (erase page)",
            Cmd::Wqw => "WQW (write quad word)",
            Cmd::Pbc => "PBC (page buffer clear)",
            Cmd::Bkswrst => "BKSWRST (bank swap and reset)",
        }
    }
}

/// `INTFLAG` bits that mean the last command failed
///
/// DONE (bit 0) is the completion flag and is polled separately.
const ERR_ADDRE: u16 = 1 << 1;
const ERR_PROGE: u16 = 1 << 2;
const ERR_LOCKE: u16 = 1 << 3;
const ERR_NVME: u16 = 1 << 6;
const ERRORS: u16 = ERR_ADDRE | ERR_PROGE | ERR_LOCKE | ERR_NVME;
/// Every flag the hardware sets, cleared by writing ones
const ALL_FLAGS: u16 = 0x07FF;

const HALT_TIMEOUT: Duration = Duration::from_millis(500);
/// Bounds the `INTFLAG.DONE` poll in [`Nvm::command`], sized for the
/// slowest command issued here, the auxiliary-row erase.
const CMD_TIMEOUT: Duration = Duration::from_millis(2000);

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Run,
    /// Reads still happen, so the plan is computed against the real part,
    /// and the core is halted for them. Writes are printed instead of
    /// issued, except the `CTRLA` cache disable in [`Nvm::uncached`], which
    /// is issued and then undone.
    DryRun,
}

impl Mode {
    pub fn writes(self) -> bool {
        matches!(self, Mode::Run)
    }
}

/// An attached debug session and the [`Mode`] its writes run under
pub struct Device {
    session: Session,
    mode: Mode,
}

impl Device {
    /// Attach to the first connected probe and select `chip` as its target
    ///
    /// `chip` is a probe-rs target name, e.g. `ATSAMD51J20A`.
    pub fn attach(chip: &str, mode: Mode) -> Result<Self> {
        let session = Session::auto_attach(chip, SessionConfig::default())
            .with_context(|| format!("attaching to {chip}"))?;
        Ok(Self { session, mode })
    }

    /// The session itself, for probe-rs operations that bring their own
    /// flash algorithm (see [`crate::flash`])
    pub fn session(&mut self) -> &mut Session {
        &mut self.session
    }

    /// Halt core 0 and hand back the NVMCTRL view of it
    ///
    /// Every NVMCTRL sequence this crate issues by hand goes through here,
    /// so the part is never running its own bootloader against the same
    /// peripheral. [`Device::session`] is the other way in, used by
    /// [`crate::flash`] for probe-rs's flash algorithm, which halts the
    /// core itself.
    pub fn halted(&mut self) -> Result<Nvm<'_>> {
        let mut core = self.session.core(0).context("selecting core 0")?;
        core.halt(HALT_TIMEOUT).context("halting the core")?;
        Ok(Nvm {
            core,
            mode: self.mode,
            leave_halted: false,
        })
    }
}

/// NVMCTRL as seen from a halted core. Dropping it runs the core again.
pub struct Nvm<'a> {
    core: Core<'a>,
    mode: Mode,
    leave_halted: bool,
}

/// Flash geometry read from `NVMCTRL.PARAM`, in bytes
#[derive(Clone, Copy)]
pub struct Param {
    pub flash_size: usize,
    pub page_size: usize,
}

#[derive(Clone, Copy)]
pub struct Status {
    /// `STATUS.BOOTPROT`, the raw fuse value: `(15 - bootprot) * 8 KiB` of
    /// the active bank is protected, so 15 protects nothing.
    pub bootprot: u8,
    /// `STATUS.AFIRST`: true when bank A is the one mapped at the flash base
    pub a_first: bool,
    /// `STATUS.BPDIS`: BOOTPROT is overridden at runtime, so the head is
    /// writable whatever the fuse says.
    pub bpdis: bool,
}

impl fmt::Display for Status {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "BOOTPROT={} ({} KiB protected), active bank {}{}",
            self.bootprot,
            (15 - self.bootprot as usize) * 8,
            if self.a_first { "A" } else { "B" },
            if self.bpdis { ", BPDIS set" } else { "" }
        )
    }
}

impl Nvm<'_> {
    pub fn param(&mut self) -> Result<Param> {
        let raw = self.core.read_word_32(PARAM).context("reading PARAM")?;
        let pages = (raw & 0xFFFF) as usize;
        // PSZ encodes the page size as a power of two starting at 8 bytes.
        let page_size = 8usize << ((raw >> 16) & 0x7);
        Ok(Param {
            flash_size: pages * page_size,
            page_size,
        })
    }

    pub fn status(&mut self) -> Result<Status> {
        let raw = self.core.read_word_16(STATUS).context("reading STATUS")?;
        Ok(Status {
            bootprot: ((raw >> 8) & 0xF) as u8,
            a_first: raw & (1 << 4) != 0,
            bpdis: raw & (1 << 5) != 0,
        })
    }

    /// Live region locks
    ///
    /// A clear bit means the region is locked.
    pub fn runlock(&mut self) -> Result<u32> {
        self.core.read_word_32(RUNLOCK).context("reading RUNLOCK")
    }

    /// `SEESTAT.SBLK`, the block count latched from the user page at reset
    pub fn see_sblk(&mut self) -> Result<u8> {
        let raw = self.core.read_word_32(SEESTAT).context("reading SEESTAT")?;
        Ok((raw & 0xF) as u8)
    }

    /// Run `f` with the NVM cache disabled, restoring `CTRLA` afterwards
    /// whether or not `f` succeeded
    ///
    /// Every read-back that checks a write must go through this: errata
    /// NVM101-7 otherwise serves the pre-erase contents, so the check
    /// passes on stale data. Programming sequences run inside it too,
    /// because the same erratum wants the cache off across a programming
    /// operation (see [`crate::provision`]).
    pub fn uncached<T>(&mut self, f: impl FnOnce(&mut Self) -> Result<T>) -> Result<T> {
        let ctrla = self.core.read_word_16(CTRLA).context("reading CTRLA")?;
        self.core
            .write_word_16(CTRLA, ctrla | CACHEDIS)
            .context("disabling the NVM cache")?;
        let out = f(self);
        self.core
            .write_word_16(CTRLA, ctrla)
            .context("restoring the NVM cache")?;
        out
    }

    /// Leave the core halted when this drops, rather than running it
    ///
    /// The default is to put the part back as it was found. A sequence that
    /// must not let the firmware run between its steps says so here.
    pub fn stay_halted(&mut self) {
        self.leave_halted = true;
    }

    /// Halt at the reset vector on the next reset, instead of running from it
    ///
    /// `BKSWRST` resets the part as part of the command, which hands control
    /// to the BOOT at the newly active bank's head before the debugger can
    /// halt it again. That BOOT reads the boot record and acts on it, and one
    /// of the things it can decide is to swap straight back: a bank recorded
    /// [`BankState::Invalid`](samd5_boot::persist::BankState::Invalid), which
    /// is what any failed download leaves behind, is one `Boot::fall_back`
    /// reverts out of. Catching the reset settles that instead of racing it.
    pub fn catch_reset(&mut self, on: bool) -> Result<()> {
        let condition = VectorCatchCondition::CoreReset;
        if on {
            self.core.enable_vector_catch(condition)
        } else {
            self.core.disable_vector_catch(condition)
        }
        .context("setting the reset vector catch")
    }

    /// Plain flash reads, for comparing a programmed image against its file
    ///
    /// The width restriction in [`Nvm::fill`] is about writes into the page
    /// buffer, not about reading the array back.
    pub fn read_bytes(&mut self, addr: u64, buf: &mut [u8]) -> Result<()> {
        self.core
            .read_8(addr, buf)
            .with_context(|| format!("reading {} bytes at {addr:#x}", buf.len()))
    }

    /// Read a run of 32-bit words, for the user page and the boot record
    pub fn read_words(&mut self, addr: u64, words: &mut [u32]) -> Result<()> {
        self.core
            .read_32(addr, words)
            .with_context(|| format!("reading {} words at {addr:#x}", words.len()))
    }

    /// Issue one NVMCTRL command and wait for `INTFLAG.DONE`
    ///
    /// Stale error flags would make the next command look like it failed,
    /// so `INTFLAG` is cleared before the command is issued and again once
    /// it completes.
    pub fn command(&mut self, cmd: Cmd) -> Result<()> {
        if !self.mode.writes() {
            println!("      CTRLB = {:#06x}  ({})", CMD_KEY | cmd as u16, cmd.name());
            return Ok(());
        }
        self.core
            .write_word_16(INTFLAG, ALL_FLAGS)
            .context("clearing INTFLAG")?;
        self.core
            .write_word_16(CTRLB, CMD_KEY | cmd as u16)
            .with_context(|| format!("issuing {}", cmd.name()))?;

        let deadline = Instant::now() + CMD_TIMEOUT;
        loop {
            let flags = self.core.read_word_16(INTFLAG).context("polling INTFLAG")?;
            if flags & ERRORS != 0 {
                self.core.write_word_16(INTFLAG, ALL_FLAGS).ok();
                bail!("{} failed: INTFLAG={flags:#06x}{}", cmd.name(), describe(flags));
            }
            if flags & 1 != 0 {
                self.core
                    .write_word_16(INTFLAG, ALL_FLAGS)
                    .context("clearing INTFLAG")?;
                return Ok(());
            }
            if Instant::now() >= deadline {
                bail!("{} did not complete within {CMD_TIMEOUT:?}", cmd.name());
            }
        }
    }

    /// Set `NVMCTRL.ADDR`, the byte address the next command acts on
    pub fn set_addr(&mut self, addr: u32) -> Result<()> {
        if !self.mode.writes() {
            println!("      ADDR = {addr:#010x}");
            return Ok(());
        }
        self.core
            .write_word_32(ADDR, addr)
            .with_context(|| format!("setting ADDR to {addr:#x}"))
    }

    /// Fill part of the page buffer
    ///
    /// A plain write to the destination address: the buffer shadows it
    /// until a commit command runs.
    ///
    /// **Words only.** DS60001507 section 25.6.3: the page buffer takes
    /// 32-bit and 64-bit writes, and nothing narrower. probe-rs's `write_8`
    /// issues byte-wide AHB transfers (`DataSize::U8`), which the buffer
    /// silently drops, so the commit that follows programs whatever the
    /// buffer already held.
    pub fn fill(&mut self, addr: u32, words: &[u32]) -> Result<()> {
        if !self.mode.writes() {
            print!("      [{addr:#010x}] <-");
            for w in words {
                print!(" {w:08x}");
            }
            println!();
            return Ok(());
        }
        self.core
            .write_32(addr as u64, words)
            .with_context(|| format!("filling the page buffer at {addr:#x}"))
    }

    /// Plain memory write, for RAM rather than the NVM page buffer
    pub fn write_words(&mut self, addr: u64, words: &[u32]) -> Result<()> {
        if !self.mode.writes() {
            println!("      [{addr:#010x}] <- {words:08x?}");
            return Ok(());
        }
        self.core
            .write_32(addr, words)
            .with_context(|| format!("writing {} words at {addr:#x}", words.len()))
    }

    /// Reset the core and halt it again
    ///
    /// The reset re-runs NVMCTRL's startup sequence, which is what re-latches
    /// BOOTPROT and the lock bits from the user page. Halting on the way out
    /// keeps the part from running whatever was just installed before the
    /// caller has verified it.
    pub fn reset(&mut self) -> Result<()> {
        if !self.mode.writes() {
            println!("      (reset)");
            return Ok(());
        }
        // SYSRESETREQ is what re-runs that startup sequence (DS60001507
        // table 16-1 lists NVMCTRL under "Others").
        self.core
            .reset_and_halt(HALT_TIMEOUT)
            .context("resetting the core")?;
        Ok(())
    }

}

impl Drop for Nvm<'_> {
    /// Leave the part running. Halting is how this type reaches NVMCTRL
    /// safely, so putting the core back is its job rather than something
    /// every exit path has to remember.
    ///
    /// Best effort: a drop cannot report. The case that fails is a core that
    /// has just reset itself, which is running anyway.
    fn drop(&mut self) {
        if !self.leave_halted {
            let _ = self.core.run();
        }
    }
}

fn describe(flags: u16) -> &'static str {
    if flags & ERR_LOCKE != 0 {
        " (LOCKE: the region is write protected)"
    } else if flags & ERR_ADDRE != 0 {
        " (ADDRE: bad address)"
    } else if flags & ERR_PROGE != 0 {
        " (PROGE: invalid command sequence)"
    } else if flags & ERR_NVME != 0 {
        " (NVME)"
    } else {
        ""
    }
}
