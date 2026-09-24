//! One attached, halted debug session, and the NVMCTRL sequences that have
//! to run inside it.
//!
//! The NVM page buffer is volatile and auxiliary pages have no
//! read-while-write, so an erase, a buffer fill and the quad-word commit
//! that follows must be one uninterrupted sequence against a core that is
//! not itself driving NVMCTRL.
//!
//! Register offsets and bit positions are DS60001507 §25.8. Command codes
//! and the user-page field positions match the PAC and the hal's
//! `RawUserpage`, so the two agree on where BOOTPROT and the lock bits live.

use std::fmt;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use probe_rs::{Core, MemoryInterface, Session, SessionConfig};

const NVMCTRL: u64 = 0x4100_4000;
const CTRLA: u64 = NVMCTRL;
const CTRLB: u64 = NVMCTRL + 0x04;
const PARAM: u64 = NVMCTRL + 0x08;
const INTFLAG: u64 = NVMCTRL + 0x10;
const STATUS: u64 = NVMCTRL + 0x12;
const ADDR: u64 = NVMCTRL + 0x14;
const RUNLOCK: u64 = NVMCTRL + 0x18;
const SEESTAT: u64 = NVMCTRL + 0x2C;

/// `CTRLA.CACHEDIS0` and `CACHEDIS1`. Errata NVM101-7 leaves the NVM cache
/// holding stale lines after programming.
const CACHEDIS: u16 = (1 << 14) | (1 << 15);

/// `CTRLB.CMDEX`: a command executes only with this key in bits 15:8.
const CMD_KEY: u16 = 0xA500;

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

/// `INTFLAG` bits that mean the last command failed. DONE (bit 0) is the
/// completion flag and is polled separately.
const ERR_ADDRE: u16 = 1 << 1;
const ERR_PROGE: u16 = 1 << 2;
const ERR_LOCKE: u16 = 1 << 3;
const ERR_NVME: u16 = 1 << 6;
const ERRORS: u16 = ERR_ADDRE | ERR_PROGE | ERR_LOCKE | ERR_NVME;
/// Every flag the hardware sets, cleared by writing ones.
const ALL_FLAGS: u16 = 0x07FF;

const HALT_TIMEOUT: Duration = Duration::from_millis(500);
/// Generous: an auxiliary-row erase is the slowest thing here, and a stall
/// is a real failure rather than something to race.
const CMD_TIMEOUT: Duration = Duration::from_millis(2000);

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Run,
    /// Reads still happen, so the plan is computed against the real part;
    /// every write is printed instead of issued.
    DryRun,
}

impl Mode {
    pub fn writes(self) -> bool {
        matches!(self, Mode::Run)
    }
}

pub struct Device {
    session: Session,
    mode: Mode,
}

impl Device {
    pub fn attach(chip: &str, mode: Mode) -> Result<Self> {
        let session = Session::auto_attach(chip, SessionConfig::default())
            .with_context(|| format!("attaching to {chip}"))?;
        Ok(Self { session, mode })
    }

    pub fn session(&mut self) -> &mut Session {
        &mut self.session
    }

    /// Halt core 0 and hand back the NVMCTRL view of it. Everything that
    /// pokes NVMCTRL goes through here, so the core is never running its own
    /// bootloader while we drive the same peripheral.
    pub fn halted(&mut self) -> Result<Nvm<'_>> {
        let mut core = self.session.core(0).context("selecting core 0")?;
        core.halt(HALT_TIMEOUT).context("halting the core")?;
        Ok(Nvm {
            core,
            mode: self.mode,
        })
    }
}

pub struct Nvm<'a> {
    core: Core<'a>,
    mode: Mode,
}

#[derive(Clone, Copy)]
pub struct Param {
    pub flash_size: usize,
    pub page_size: usize,
}

#[derive(Clone, Copy)]
pub struct Status {
    pub bootprot: u8,
    /// `STATUS.AFIRST`: true when bank A is the one mapped at the flash base.
    pub a_first: bool,
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

    /// Live region locks. A clear bit means the region is locked.
    pub fn runlock(&mut self) -> Result<u32> {
        self.core.read_word_32(RUNLOCK).context("reading RUNLOCK")
    }

    /// `SEESTAT.SBLK`, the SmartEEPROM blocks the hardware came up with.
    pub fn see_sblk(&mut self) -> Result<u8> {
        let raw = self.core.read_word_32(SEESTAT).context("reading SEESTAT")?;
        Ok((raw & 0xF) as u8)
    }

    /// Read with the NVM cache off, then restore it. Anything read back to
    /// check a write must go through this: errata NVM101-7 otherwise serves
    /// the pre-erase contents and every verification passes for free.
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

    /// Plain flash reads, for comparing a programmed image against its file.
    /// The width restriction in [`Nvm::fill`] is about writes into the page
    /// buffer, not about reading the array back.
    pub fn read_bytes(&mut self, addr: u64, buf: &mut [u8]) -> Result<()> {
        self.core
            .read_8(addr, buf)
            .with_context(|| format!("reading {} bytes at {addr:#x}", buf.len()))
    }

    pub fn read_words(&mut self, addr: u64, words: &mut [u32]) -> Result<()> {
        self.core
            .read_32(addr, words)
            .with_context(|| format!("reading {} words at {addr:#x}", words.len()))
    }

    /// Issue one NVMCTRL command and wait for it.
    ///
    /// Stale error flags make the next command look like it failed, so they
    /// are cleared first; this mirrors the hal's `command_sync`.
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

    /// `NVMCTRL.ADDR`, which selects what the next command acts on. In
    /// auxiliary space this is a plain byte address, not a word index.
    pub fn set_addr(&mut self, addr: u32) -> Result<()> {
        if !self.mode.writes() {
            println!("      ADDR = {addr:#010x}");
            return Ok(());
        }
        self.core
            .write_word_32(ADDR, addr)
            .with_context(|| format!("setting ADDR to {addr:#x}"))
    }

    /// Fill part of the page buffer. A plain write to the destination
    /// address: the buffer shadows it until a commit command runs.
    ///
    /// **Words only.** DS 25.6.3: the page buffer takes 32-bit and 64-bit
    /// writes, and nothing narrower. probe-rs's `write_8` issues byte-wide AHB
    /// transfers (`DataSize::U8`), which the buffer silently drops, so the
    /// commit that follows programs whatever the buffer already held.
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

    /// Plain memory write, for RAM rather than the NVM page buffer.
    pub fn write_words(&mut self, addr: u64, words: &[u32]) -> Result<()> {
        if !self.mode.writes() {
            println!("      [{addr:#010x}] <- {words:08x?}");
            return Ok(());
        }
        self.core
            .write_32(addr, words)
            .with_context(|| format!("writing {} words at {addr:#x}", words.len()))
    }

    pub fn reset(&mut self) -> Result<()> {
        if !self.mode.writes() {
            println!("      (reset)");
            return Ok(());
        }
        // SYSRESETREQ re-runs the NVMCTRL startup sequence, which is what
        // re-latches BOOTPROT and the lock bits from the user page
        // (DS Table 16-1 lists NVMCTRL under "Others"). Halting on the way
        // back out keeps the part from running whatever was just installed
        // before the caller has verified it.
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
    /// Best effort, because a drop cannot report. The one case that fails
    /// here is a core that has just reset itself out from under us, which
    /// is running anyway.
    fn drop(&mut self) {
        let _ = self.core.run();
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
