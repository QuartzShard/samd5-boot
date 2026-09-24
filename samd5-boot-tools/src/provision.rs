//! Fuse provisioning: the user-page write that decides what BOOTPROT
//! protects and which flash regions are locked
//!
//! The page is one erase-and-rewrite unit, so every field in it is read,
//! patched and written back together. Fields not named on the command line
//! are preserved word for word, which matters because the page also carries
//! the BOD levels, the watchdog fuses and the SmartEEPROM configuration.
//!
//! Field positions come from the hal's `RawUserpage` and the NVM User Page
//! Mapping table in DS60001507 section 25: BOOTPROT at bits 29:26, SEE SBLK
//! at 35:32, and NVM LOCKS at 95:64, where a *clear* bit locks the region.
//!
//! # The erase window
//!
//! Between the page erase and the last quad-word commit the part has no
//! fuses at all: an erased page reads 0xFF everywhere, which decodes as
//! BOOTPROT 15 (nothing protected) and SEE SBLK 15 (a reserved value that
//! `Boot::new` rejects). Nothing is destroyed, since the page can simply be
//! written again, but the contents have to come from somewhere. So the page
//! is saved to a backup file before the erase, and `--restore` writes one
//! back verbatim.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use samd5_boot::consts::{USER_PAGE_ADDR, USER_PAGE_SIZE, WRITE_UNIT, geometry};

use crate::probe::{Cmd, Device, Mode, Nvm};

const PAGE_WORDS: usize = USER_PAGE_SIZE / 4;
const QUAD_WORDS: usize = WRITE_UNIT / 4;

/// The user page as stored, with accessors for the fields this tool touches
///
/// Held as words because the page buffer takes word-wide writes and nothing
/// narrower (see [`Nvm::fill`]).
#[derive(Clone, PartialEq, Eq)]
pub struct UserPage([u32; PAGE_WORDS]);

impl UserPage {
    pub fn bootprot(&self) -> u8 {
        ((self.0[0] >> 26) & 0xF) as u8
    }

    pub fn set_bootprot(&mut self, value: u8) {
        self.0[0] = (self.0[0] & !(0xF << 26)) | ((value as u32 & 0xF) << 26);
    }

    pub fn see_sblk(&self) -> u8 {
        (self.0[1] & 0xF) as u8
    }

    pub fn set_see_sblk(&mut self, blocks: u8) {
        self.0[1] = (self.0[1] & !0xF) | (blocks as u32 & 0xF);
    }

    /// One bit per flash region; a clear bit locks that region
    pub fn locks(&self) -> u32 {
        self.0[2]
    }

    pub fn set_locks(&mut self, locks: u32) {
        self.0[2] = locks;
    }

    fn is_blank(&self) -> bool {
        self.0.iter().all(|&w| w == 0xFFFF_FFFF)
    }

    fn quad(&self, i: usize) -> &[u32] {
        &self.0[i * QUAD_WORDS..(i + 1) * QUAD_WORDS]
    }

    fn to_bytes(&self) -> Vec<u8> {
        self.0.iter().flat_map(|w| w.to_le_bytes()).collect()
    }

    fn from_bytes(bytes: &[u8]) -> Result<Self> {
        if bytes.len() != USER_PAGE_SIZE {
            bail!("a user page is {USER_PAGE_SIZE} bytes, got {}", bytes.len());
        }
        let mut words = [0u32; PAGE_WORDS];
        let (chunks, _) = bytes.as_chunks::<4>();
        for (w, chunk) in words.iter_mut().zip(chunks) {
            *w = u32::from_le_bytes(*chunk);
        }
        Ok(Self(words))
    }

    /// Where this page first differs from another, for reporting a failed
    /// verification as something specific rather than "differs"
    fn first_difference(&self, other: &Self) -> Option<usize> {
        self.0.iter().zip(&other.0).position(|(a, b)| a != b)
    }
}

/// Read the user page with the NVM cache disabled
pub fn read_page(nvm: &mut Nvm<'_>) -> Result<UserPage> {
    nvm.uncached(|nvm| {
        let mut words = [0u32; PAGE_WORDS];
        nvm.read_words(USER_PAGE_ADDR as u64, &mut words)?;
        Ok(UserPage(words))
    })
}

/// Erase the user page and write `page` back into it
///
/// The erase and every quad-word commit happen against one halted core
/// inside one attach, so the volatile page buffer survives from fill to
/// commit and nothing else is driving NVMCTRL in between. Errata 2.14.1
/// also wants the NVM cache off across a programming operation, which is
/// what [`Nvm::uncached`] gives here.
fn write_page(nvm: &mut Nvm<'_>, page: &UserPage) -> Result<()> {
    nvm.uncached(|nvm| {
        nvm.set_addr(USER_PAGE_ADDR as u32)?;
        nvm.command(Cmd::Ep)?;

        for quad in 0..USER_PAGE_SIZE / WRITE_UNIT {
            // The erase left every quad word at 0xFF, so one that is already
            // 0xFF is in its final state; programming it would only spend
            // endurance.
            let words = page.quad(quad);
            if words.iter().all(|&w| w == 0xFFFF_FFFF) {
                continue;
            }
            let addr = (USER_PAGE_ADDR + quad * WRITE_UNIT) as u32;
            nvm.command(Cmd::Pbc)?;
            nvm.fill(addr, words)?;
            // Explicit, because a page-buffer write advances ADDR by itself
            // and WQW commits whatever ADDR points at.
            nvm.set_addr(addr)?;
            nvm.command(Cmd::Wqw)?;
        }
        Ok(())
    })
}

/// The fuse settings [`run`] computes a page from
pub struct Fuses {
    /// BOOT region size in bytes. Must satisfy
    /// [`geometry::boot_size_valid`] for the part's flash density.
    pub boot_size: usize,
    /// Lock the BOOT regions of both banks, so neither copy can be erased
    /// or written after reset.
    pub lock_boot: bool,
    /// SmartEEPROM blocks, 0..=10. `None` leaves the field as it is.
    pub see_sblk: Option<u8>,
}

/// What [`run`] writes: a page computed from [`Fuses`], or a saved page put
/// back word for word
pub enum Request {
    Fuses(Fuses),
    Restore(PathBuf),
}

/// Provision one part: read the user page, plan the change, write it, and
/// check it survived a reset
///
/// The page is read first, so `req` is applied to what the part actually
/// holds, and a page that already holds the result is left untouched. The
/// previous contents are saved under `backup_dir` before the erase, unless
/// the page was blank or a backup is already there. After the write the part
/// is reset so NVMCTRL re-latches the fuses, and `STATUS.BOOTPROT` and
/// `RUNLOCK` are compared against what was asked for.
///
/// In [`Mode::DryRun`] the plan is printed and nothing is backed up or
/// written.
pub fn run(chip: &str, req: Request, backup_dir: &Path, mode: Mode) -> Result<()> {
    let mut device = Device::attach(chip, mode)?;
    let mut nvm = device.halted()?;

    let param = nvm.param()?;
    println!(
        "part reports {} KiB flash, {} B pages",
        param.flash_size / 1024,
        param.page_size
    );

    let before = read_page(&mut nvm)?;
    if before.is_blank() {
        println!(
            "note: the user page is blank, so this part currently has no fuses \
             (BOOTPROT 15, SEE SBLK 15)"
        );
    }

    let after = match &req {
        Request::Restore(path) => {
            let bytes = std::fs::read(path)
                .with_context(|| format!("reading {}", path.display()))?;
            let page = UserPage::from_bytes(&bytes)?;
            println!(
                "restoring {} verbatim: BOOTPROT={} SEE SBLK={} LOCKS={:#010x}",
                path.display(),
                page.bootprot(),
                page.see_sblk(),
                page.locks()
            );
            page
        }
        Request::Fuses(fuses) => plan(&before, fuses, param.flash_size)?,
    };

    if after == before {
        println!("user page already holds this; nothing to write");
        return Ok(());
    }

    if !mode.writes() {
        println!("writing the user page");
        write_page(&mut nvm, &after)?;
        println!("dry run: nothing was written");
        return Ok(());
    }

    let backup = save_backup(backup_dir, chip, &before)?;
    println!("writing the user page");
    let outcome = write_page(&mut nvm, &after).and_then(|()| verify(&mut nvm, &after));
    if outcome.is_err() {
        match &backup {
            Some(path) => eprintln!(
                "\nthe page was erased before this failed, so the part may now have no \
                 fuses. The previous contents are in {}; put them back with:\n  \
                 cargo xtask provision --chip {chip} --restore {}",
                path.display(),
                path.display()
            ),
            None => eprintln!(
                "\nthe page was erased before this failed, so the part may now have no \
                 fuses. It was blank before this ran, so there is nothing to put back: \
                 provision it again."
            ),
        }
    }
    outcome?;

    let status = nvm.status()?;
    let runlock = nvm.runlock()?;
    println!("after reset: {status}");
    println!("  RUNLOCK {runlock:#010x} (clear bits are locked regions)");
    if status.bootprot != after.bootprot() {
        bail!(
            "BOOTPROT latched as {} but the page says {}",
            status.bootprot,
            after.bootprot()
        );
    }
    if let Request::Fuses(fuses) = &req
        && fuses.lock_boot
    {
        let boot_regions = geometry::boot_region_mask(param.flash_size, fuses.boot_size);
        if runlock & boot_regions != 0 {
            bail!(
                "BOOT regions did not latch as locked: RUNLOCK {runlock:#010x} \
                 still has bits of {boot_regions:#010x} set"
            );
        }
    }
    println!("provisioned and verified");
    Ok(())
}

/// What the requested options make of the page that is there now
fn plan(before: &UserPage, req: &Fuses, flash_size: usize) -> Result<UserPage> {
    if !geometry::boot_size_valid(flash_size, req.boot_size) {
        bail!(
            "a {} KiB BOOT is not usable on a {} KiB part: it must be a whole number of \
             {} KiB lock regions, fit inside a {} KiB bank, and be expressible in BOOTPROT",
            req.boot_size / 1024,
            flash_size / 1024,
            geometry::lock_region_size(flash_size) / 1024,
            geometry::bank_size(flash_size) / 1024,
        );
    }

    let mut after = before.clone();
    let want_bootprot = geometry::bootprot_value(req.boot_size);
    after.set_bootprot(want_bootprot);

    let boot_regions = geometry::boot_region_mask(flash_size, req.boot_size);
    let locks = if req.lock_boot {
        before.locks() & !boot_regions
    } else {
        before.locks() | boot_regions
    };
    after.set_locks(locks);

    if let Some(sblk) = req.see_sblk {
        if sblk > 10 {
            bail!("SEE SBLK must be 0..=10, got {sblk}");
        }
        after.set_see_sblk(sblk);
    }

    println!(
        "  BOOTPROT {} -> {} ({} KiB)",
        before.bootprot(),
        want_bootprot,
        req.boot_size / 1024
    );
    println!(
        "  NVM LOCKS {:#010x} -> {:#010x} (BOOT regions {:#010x}, {})",
        before.locks(),
        locks,
        boot_regions,
        if req.lock_boot { "locked" } else { "unlocked" }
    );
    if let Some(sblk) = req.see_sblk {
        println!("  SEE SBLK {} -> {}", before.see_sblk(), sblk);
    }
    Ok(after)
}

/// Check the write landed, then reset so NVMCTRL re-latches the fuses
///
/// The caller is what compares the latched values against STATUS and
/// RUNLOCK.
fn verify(nvm: &mut Nvm<'_>, want: &UserPage) -> Result<()> {
    let stored = read_page(nvm)?;
    if let Some(w) = stored.first_difference(want) {
        bail!(
            "the write did not land: word {w} ({:#x}) reads {:#010x}, expected {:#010x}",
            USER_PAGE_ADDR + w * 4,
            stored.0[w],
            want.0[w]
        );
    }
    println!("  page contents verified");

    // The fuses only take effect once NVMCTRL's startup sequence re-reads
    // them, so the latched values are only meaningful after a reset.
    nvm.reset()?;
    let after_reset = read_page(nvm)?;
    if let Some(w) = after_reset.first_difference(want) {
        bail!(
            "the page changed across the reset: word {w} reads {:#010x}, expected {:#010x}",
            after_reset.0[w],
            want.0[w]
        );
    }
    Ok(())
}

/// Keep the pre-erase contents, once
///
/// A second failed run must not overwrite the good capture with the blank
/// page the first one left behind.
///
/// `None` means no file holds the previous contents, because a blank page has
/// none worth keeping.
fn save_backup(dir: &Path, chip: &str, page: &UserPage) -> Result<Option<PathBuf>> {
    let path = dir.join(format!("userpage-{chip}.bak"));
    if path.exists() {
        println!("  keeping the existing backup at {}", path.display());
        return Ok(Some(path));
    }
    if page.is_blank() {
        println!("  not backing up a blank page");
        return Ok(None);
    }
    std::fs::write(&path, page.to_bytes())
        .with_context(|| format!("writing {}", path.display()))?;
    println!("  saved the current user page to {}", path.display());
    Ok(Some(path))
}

/// Read-only report, for finding out what a part is actually configured as
/// before changing anything
pub fn info(chip: &str) -> Result<()> {
    let mut device = Device::attach(chip, Mode::Run)?;
    let mut nvm = device.halted()?;

    let param = nvm.param()?;
    let status = nvm.status()?;
    let page = read_page(&mut nvm).context("reading the user page")?;

    println!("{chip}");
    println!(
        "  flash        {} KiB, {} B pages, {} KiB banks",
        param.flash_size / 1024,
        param.page_size,
        geometry::bank_size(param.flash_size) / 1024
    );
    println!("  status       {status}");
    println!(
        "  user page    BOOTPROT={} SEE SBLK={} LOCKS={:#010x}{}",
        page.bootprot(),
        page.see_sblk(),
        page.locks(),
        if page.is_blank() { "  (BLANK)" } else { "" }
    );
    println!("  RUNLOCK      {:#010x}", nvm.runlock()?);
    println!("  SEESTAT.SBLK {}", nvm.see_sblk()?);
    Ok(())
}

/// Set the update request through the debugger, so BOOT waits for an image
/// instead of booting the application
///
/// The application is what normally sets this flag before resetting, so an
/// application that cannot be asked (wrong transport, wedged, or absent
/// from the protocol) would leave no way into the update window. The flag
/// lives in the boot record, which a debugger can write as easily as the
/// application can.
///
/// The record is read, amended and resealed rather than replaced, so the
/// trial bookkeeping already in it survives.
pub fn request_update(chip: &str, record_addr: u64, mode: Mode) -> Result<()> {
    use samd5_boot::persist::BootStore;

    let addr = record_addr;
    let words = size_of::<BootStore>() / 4;

    let mut device = Device::attach(chip, mode)?;
    let mut nvm = device.halted()?;

    let mut raw = vec![0u32; words];
    nvm.read_words(addr, &mut raw)?;
    let bytes: Vec<u8> = raw.iter().flat_map(|w| w.to_le_bytes()).collect();

    let stored: BootStore = bytemuck::pod_read_unaligned(&bytes);
    let mut record = match stored.validated() {
        Some(r) => {
            println!("boot record at {addr:#x} checks out, amending it");
            r
        }
        None => {
            // Backup RAM does not survive a power cut, so an unchecksummed
            // record is the normal state on a cold part rather than damage.
            println!("no valid boot record at {addr:#x}, writing a fresh one");
            BootStore::default()
        }
    };
    record.mailbox.set_request_update(true);
    record.seal();

    let out = bytemuck::bytes_of(&record);
    let patched: Vec<u32> = out
        .as_chunks::<4>()
        .0
        .iter()
        .map(|c| u32::from_le_bytes(*c))
        .collect();

    nvm.write_words(addr, &patched)?;
    println!(
        "{} the update request; resetting into BOOT",
        if mode.writes() { "set" } else { "would set" }
    );
    nvm.reset()?;
    Ok(())
}
