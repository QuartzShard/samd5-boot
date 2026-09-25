//! Place BOOT at the head of both banks
//!
//! [`run`] is the whole module: it reads `STATUS`, writes both heads, and
//! reads them back against the file.
//!
//! BOOTPROT protects only the head of the *active* bank, so on a part whose
//! BOOTPROT is set one of the two copies cannot be written where it sits.
//! The way round is the swap: write the inactive head, `BKSWRST`, write the
//! head that is now inactive, swap back. Two swaps leave the same bank
//! active as before, so the application that was running is still the one
//! that will run. With BOOTPROT at 15 or `STATUS.BPDIS` set, both heads are
//! written where they are.
//!
//! Issuing `BKSWRST` from the debugger, rather than asking a cooperating
//! application to do it, is what makes this work on a part whose
//! application is missing or broken.
//!
//! `BKSWRST` also resets the part, which hands control to the BOOT at the
//! head that is now active. That BOOT reads the boot record and acts on it,
//! and one of the things it can decide is to swap straight back: a bank
//! recorded `Invalid`, which is what any failed download leaves behind, is
//! one `Boot::fall_back` reverts out of. So the swap arms the reset vector
//! catch first and holds the core halted until both heads are written and
//! read back, rather than racing the firmware for the bank. `release` is
//! the single place that undoes it.
//!
//! Region locks are a separate protection from BOOTPROT:
//! [`crate::provision`] locks the BOOT regions of *both* banks unless asked
//! not to, and nothing here reads `RUNLOCK` or unlocks anything.

use std::path::Path;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use probe_rs::flashing::{BinOptions, DownloadOptions, Format, download_file_with_options};
use samd5_boot::consts::geometry;

use crate::probe::{Cmd, Device, Mode};

/// Long enough for the part to come back from a BKSWRST and be halted again
const RESWAP_TIMEOUT: Duration = Duration::from_secs(3);

/// Place `boot_bin` at the head of both banks and read both back
///
/// `boot_bin` is a raw binary written at each bank base, not an ELF. Only
/// the BOOT region is erased; the application in the rest of each bank is
/// left alone. Fails if either head does not read back as the file.
pub fn run(chip: &str, boot_bin: &Path, mode: Mode) -> Result<()> {
    let image = std::fs::read(boot_bin)
        .with_context(|| format!("reading {}", boot_bin.display()))?;
    let mut device = Device::attach(chip, mode)?;

    let (flash_size, status) = {
        let mut nvm = device.halted()?;
        (nvm.param()?.flash_size, nvm.status()?)
    };
    let inactive = geometry::bank_size(flash_size) as u64;
    println!("{status}");
    println!(
        "placing {} ({} bytes) at both bank heads",
        boot_bin.display(),
        image.len()
    );

    // BOOTPROT 15 is "nothing protected", and BPDIS is a runtime override of
    // whatever it says; either way both heads are writable where they are.
    let protected = status.bootprot != 15 && !status.bpdis;

    if !mode.writes() {
        if protected {
            println!(
                "dry run: would write {inactive:#x}, swap, write {inactive:#x} again, swap back"
            );
        } else {
            println!("dry run: would write 0x0 and {inactive:#x} directly");
        }
        return Ok(());
    }

    if !protected {
        download(&mut device, boot_bin, 0)?;
        download(&mut device, boot_bin, inactive)?;
    } else {
        println!("  active head is BOOTPROT protected, going round by the swap");
        download(&mut device, boot_bin, inactive)?;
        swap(&mut device)?;
        download(&mut device, boot_bin, inactive)?;
        swap(&mut device)?;
    }

    verify(&mut device, &image, 0)?;
    verify(&mut device, &image, inactive)?;
    release(&mut device)?;
    println!("both bank heads carry this BOOT");

    Ok(())
}

/// Put the part back on its feet: reset catch off, core running.
///
/// The swap holds the core halted at the reset vector for the length of the
/// dance, and every step in between inherits that. This is the one place
/// that undoes it, so a `flash` always leaves the part running whichever
/// branch it took.
fn release(device: &mut Device) -> Result<()> {
    let mut nvm = device.halted()?;
    nvm.catch_reset(false)?;
    Ok(())
}

fn download(device: &mut Device, bin: &Path, base_address: u64) -> Result<()> {
    println!("  writing {base_address:#x}");
    let mut options = DownloadOptions::default();
    // Only the BOOT region is being replaced; the application in the rest of
    // the bank is not ours to erase.
    options.do_chip_erase = false;
    options.verify = true;
    download_file_with_options(
        device.session(),
        bin,
        Format::Bin(BinOptions {
            base_address: Some(base_address),
            skip: 0,
        }),
        options,
    )
    .with_context(|| format!("writing {} at {base_address:#x}", bin.display()))?;
    Ok(())
}

/// Swap the banks and wait for the part to come back halted
///
/// BKSWRST resets as part of the swap, so the core is gone the moment the
/// command takes; there is nothing to poll on the far side except the halt
/// itself.
fn swap(device: &mut Device) -> Result<()> {
    {
        let mut nvm = device.halted()?;
        let before = nvm.status()?;
        println!(
            "  swapping banks (active {} -> {})",
            if before.a_first { "A" } else { "B" },
            if before.a_first { "B" } else { "A" }
        );
        // The reset BKSWRST performs hands control to the BOOT at the head
        // that is now active, which reads the boot record and can revert
        // straight back out of a bank recorded `Invalid`. Catch the reset
        // and hold the core there rather than racing it for the bank.
        nvm.catch_reset(true)?;
        nvm.stay_halted();
        // Errors show up as the re-halt below failing.
        nvm.command(Cmd::Bkswrst).ok();
    }

    let deadline = Instant::now() + RESWAP_TIMEOUT;
    loop {
        match device.halted() {
            Ok(mut nvm) => {
                // Held halted at the reset vector until the dance is done,
                // so nothing in between gets to swap the banks back.
                nvm.stay_halted();
                nvm.catch_reset(false)?;
                let after = nvm.status()?;
                println!("    back up, active bank {}", if after.a_first { "A" } else { "B" });
                return Ok(());
            }
            Err(e) if Instant::now() >= deadline => {
                return Err(e).context("the part did not come back halted after BKSWRST");
            }
            Err(_) => {}
        }
    }
}

fn verify(device: &mut Device, image: &[u8], base: u64) -> Result<()> {
    let which = if base == 0 { "active" } else { "inactive" };
    let mut nvm = device.halted()?;
    // Which head is which is only stable while nothing is running to swap
    // them; `release` puts the part back on its feet once both are read.
    nvm.stay_halted();
    let read = nvm.uncached(|nvm| {
        let mut buf = vec![0u8; image.len()];
        nvm.read_bytes(base, &mut buf)?;
        Ok(buf)
    })?;
    if read != image {
        let at = read
            .iter()
            .zip(image)
            .position(|(a, b)| a != b)
            .unwrap_or(0);
        bail!(
            "{which} head at {base:#x} differs from the image, first at byte {at} \
             ({:#04x} on the part, {:#04x} in the file)",
            read[at],
            image[at]
        );
    }
    println!("  {which} head at {base:#x} verified ({} bytes)", image.len());
    Ok(())
}
