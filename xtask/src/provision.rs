//! Fuse provisioning: the user-page write that decides what BOOTPROT
//! protects and which flash regions are locked.
//!
//! The page is one erase-and-rewrite unit, so every field in it is read,
//! patched and written back together. Fields not named on the command line
//! are preserved word for word, which matters because the page also carries
//! the BOD levels, the watchdog fuses and the SmartEEPROM configuration.
//!
//! Field positions come from the hal's `RawUserpage` and DS60001507 Table
//! 25-6: BOOTPROT at bits 29:26, SEE SBLK at 35:32, SEE PSZ at 38:36, and
//! NVM LOCKS at 95:64 where a *clear* bit locks the region.
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

use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use samd5_boot::consts::{USER_PAGE_ADDR, USER_PAGE_SIZE, WRITE_UNIT, geometry};

use crate::probe::{Device, Mode, Nvm, cmd};

const PAGE_WORDS: usize = USER_PAGE_SIZE / 4;
const QUAD_WORDS: usize = WRITE_UNIT / 4;

/// The page as stored, with accessors for the fields this tool touches.
/// Held as words because that is the only width NVM accepts.
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

    /// One bit per flash region; a clear bit locks that region.
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
    /// verification as something specific rather than "differs".
    fn first_difference(&self, other: &Self) -> Option<usize> {
        self.0.iter().zip(&other.0).position(|(a, b)| a != b)
    }
}

pub fn read_page(nvm: &mut Nvm<'_>) -> Result<UserPage> {
    nvm.uncached(|nvm| {
        let mut words = [0u32; PAGE_WORDS];
        nvm.read_words(USER_PAGE_ADDR as u64, &mut words)?;
        Ok(UserPage(words))
    })
}

/// Erase the user page and write `page` back into it.
///
/// The erase and every quad-word commit happen against one halted core
/// inside one attach, so the volatile page buffer survives from fill to
/// commit and nothing else is driving NVMCTRL in between. Errata 2.14.1
/// also wants the NVM cache off across a programming operation, which is
/// what `uncached` gives here.
fn write_page(nvm: &mut Nvm<'_>, page: &UserPage) -> Result<()> {
    nvm.uncached(|nvm| {
        nvm.set_addr(USER_PAGE_ADDR as u32)?;
        nvm.command(cmd::EP)?;

        for quad in 0..USER_PAGE_SIZE / WRITE_UNIT {
            // The erase left every quad word at 0xFF, so one that is already
            // 0xFF is in its final state; programming it would only spend
            // endurance.
            let words = page.quad(quad);
            if words.iter().all(|&w| w == 0xFFFF_FFFF) {
                continue;
            }
            let addr = (USER_PAGE_ADDR + quad * WRITE_UNIT) as u32;
            nvm.command(cmd::PBC)?;
            nvm.fill(addr, words)?;
            // Explicit, because a page-buffer write advances ADDR by itself
            // and WQW commits whatever ADDR points at.
            nvm.set_addr(addr)?;
            nvm.command(cmd::WQW)?;
        }
        Ok(())
    })
}

pub struct Request {
    pub boot_size: usize,
    pub lock_boot: bool,
    pub see_sblk: Option<u8>,
    pub restore: Option<PathBuf>,
}

pub fn run(chip: &str, req: Request, mode: Mode) -> Result<()> {
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

    let after = match &req.restore {
        Some(path) => {
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
        None => plan(&before, &req, param.flash_size)?,
    };

    if after == before {
        println!("user page already holds this; nothing to write");
        return Ok(());
    }

    let backup = if mode.writes() {
        Some(save_backup(chip, &before)?)
    } else {
        None
    };
    println!("writing the user page");
    let Some(backup) = backup else {
        write_page(&mut nvm, &after)?;
        println!("dry run: nothing was written");
        return Ok(());
    };
    let outcome = write_page(&mut nvm, &after).and_then(|()| verify(&mut nvm, &after));
    if outcome.is_err() {
        eprintln!(
            "\nthe page was erased before this failed, so the part may now have no \
             fuses. The previous contents are in {}; put them back with:\n  \
             cargo xtask provision --chip {chip} --restore {}",
            backup.display(),
            backup.display()
        );
    }
    outcome?;

    let boot_regions = geometry::boot_region_mask(param.flash_size, req.boot_size);
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
    if req.restore.is_none() && req.lock_boot && runlock & boot_regions != 0 {
        bail!(
            "BOOT regions did not latch as locked: RUNLOCK {runlock:#010x} \
             still has bits of {boot_regions:#010x} set"
        );
    }
    println!("provisioned and verified");
    Ok(())
}

/// What the requested options make of the page that is there now.
fn plan(before: &UserPage, req: &Request, flash_size: usize) -> Result<UserPage> {
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

/// Check the write landed, then reset so NVMCTRL re-latches the fuses. The
/// caller is what compares the latched values against STATUS and RUNLOCK.
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

/// Keep the pre-erase contents, once. A second failed run must not overwrite
/// the good capture with the blank page the first one left behind.
fn save_backup(chip: &str, page: &UserPage) -> Result<PathBuf> {
    let path = crate::image::repo_root().join(format!("userpage-{chip}.bak"));
    if path.exists() {
        println!("  keeping the existing backup at {}", path.display());
        return Ok(path);
    }
    if page.is_blank() {
        println!("  not backing up a blank page");
        return Ok(path);
    }
    std::fs::write(&path, page.to_bytes())
        .with_context(|| format!("writing {}", path.display()))?;
    println!("  saved the current user page to {}", path.display());
    Ok(path)
}

/// Read-only report, for finding out what a part is actually configured as
/// before changing anything.
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
/// instead of booting the application.
///
/// The application is what normally sets this flag before resetting, so an
/// application that cannot be asked (wrong transport, wedged, or absent
/// from the protocol) would leave no way into the update window. The flag
/// lives in the boot record, which a debugger can write as easily as the
/// application can.
///
/// The record is read, amended and resealed rather than replaced, so the
/// trial bookkeeping already in it survives.
pub fn request_update(chip: &str, offset: usize, mode: Mode) -> Result<()> {
    use samd5_boot::consts::BKUPRAM_ADDR;
    use samd5_boot::persist::BootStore;

    let addr = (BKUPRAM_ADDR + offset) as u64;
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

    if !mode.writes() {
        println!("dry run: would write {patched:08x?} to {addr:#x}");
        return Ok(());
    }
    nvm.write_words(addr, &patched)?;
    println!("update request set; resetting into BOOT");
    nvm.reset()?;
    Ok(())
}
