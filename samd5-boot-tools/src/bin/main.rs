//! Drive a samd5-boot device by hand: read its fuses, write them, place a
//! bootloader at both bank heads, stamp an image.
//!
//! Everything here is a thin front end over the library, which is the form to
//! use from a project's own `xtask` where it can share a build.

use std::path::PathBuf;
use std::process::ExitCode;

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use samd5_boot_tools::{Mode, flash, image, provision};

#[derive(Parser)]
#[command(name = "samd5-boot-tools", about = "Bench tooling for samd5-boot", version)]
struct Cli {
    /// Target as probe-rs names it, e.g. ATSAMD51J20A.
    #[arg(long, global = true, default_value = "ATSAMD51J20A")]
    chip: String,
    /// Print what would be written instead of writing it. Reads still happen,
    /// so the plan is computed against the real part.
    #[arg(long, global = true)]
    dry_run: bool,
    #[command(subcommand)]
    command: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Report what a part is configured as, without changing anything.
    Info,

    /// Write the BOOTPROT and region-lock fuses, then verify them after a
    /// reset.
    Provision {
        /// BOOT region size. Must match the `bootprot-*` feature the firmware
        /// was built with.
        #[arg(long, default_value = "32k", value_parser = parse_size)]
        boot_size: usize,
        /// Leave the BOOT regions unlocked rather than write protecting both
        /// copies from power-on.
        #[arg(long)]
        no_lock: bool,
        /// SmartEEPROM blocks (SBLK, 0..=10). Left alone when not given.
        #[arg(long)]
        sblk: Option<u8>,
        /// Where to keep the pre-erase copy of the user page.
        #[arg(long, default_value = ".")]
        backup_dir: PathBuf,
        /// Write a saved user page back verbatim, ignoring the other options.
        #[arg(long)]
        restore: Option<PathBuf>,
    },

    /// Place a bootloader image at the head of both banks.
    Flash { boot: PathBuf },

    /// Stamp a manifest into a linked application image.
    Stamp {
        input: PathBuf,
        output: PathBuf,
        #[arg(long)]
        version: u16,
    },

    /// Set the update request in the boot record, so the bootloader waits for
    /// an image instead of booting the application.
    ///
    /// The address is where the firmware keeps its record, which depends on
    /// the store it uses. Only a directly addressable store can be written
    /// this way: a SmartEEPROM record goes through NVMCTRL and cannot be
    /// poked.
    RequestUpdate {
        #[arg(long, value_parser = parse_addr)]
        record_addr: u64,
    },
}

fn parse_size(s: &str) -> Result<usize, String> {
    let t = s.trim().to_ascii_lowercase();
    let (digits, scale) = match t.strip_suffix('k') {
        Some(d) => (d, 1024),
        None => (t.as_str(), 1),
    };
    digits
        .parse::<usize>()
        .map(|n| n * scale)
        .map_err(|_| format!("expected a size like 32k or 32768, got {s:?}"))
}

fn parse_addr(s: &str) -> Result<u64, String> {
    let t = s.trim();
    let parsed = match t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        Some(hex) => u64::from_str_radix(hex, 16),
        None => t.parse(),
    };
    parsed.map_err(|_| format!("expected an address like 0x47000000, got {s:?}"))
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e:#}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<()> {
    let cli = Cli::parse();
    let mode = if cli.dry_run { Mode::DryRun } else { Mode::Run };
    let chip = cli.chip.as_str();

    match cli.command {
        Cmd::Info => provision::info(chip),

        Cmd::Provision {
            boot_size,
            no_lock,
            sblk,
            backup_dir,
            restore,
        } => {
            let request = match restore {
                Some(path) => provision::Request::Restore(path),
                None => provision::Request::Fuses(provision::Fuses {
                    boot_size,
                    lock_boot: !no_lock,
                    see_sblk: sblk,
                }),
            };
            provision::run(chip, request, &backup_dir, mode)
        }

        Cmd::Flash { boot } => flash::run(chip, &boot, mode),

        Cmd::Stamp {
            input,
            output,
            version,
        } => {
            let stamped = image::stamp(&input, &output, version)
                .with_context(|| format!("stamping {}", input.display()))?;
            println!(
                "stamped {} ({} bytes): version={} crc32_vec_table={:#010x} crc32_image={:#010x}",
                output.display(),
                stamped.image_len,
                stamped.version,
                stamped.crc32_vec_table,
                stamped.crc32_image
            );
            Ok(())
        }

        Cmd::RequestUpdate { record_addr } => provision::request_update(chip, record_addr, mode),
    }
}
