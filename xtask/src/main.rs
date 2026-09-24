//! Bench tooling for samd5-boot: build and stamp images, place BOOT at both
//! bank heads, provision the fuses, and run the rig self-test.
//!
//! Everything here shares the library's own definition of the flash ABI and
//! the fuse encodings (`samd5-boot` with `--no-default-features`), so there
//! is no second implementation of a CRC range or a BOOTPROT value to drift.

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};
use proto::{Message, Status};

mod image;
mod link;
mod selftest;

use link::{Link, Transport, rtt::RttLink, serial::Serial};
use samd5_boot_tools::{Mode, flash, image as tools_image, provision};

#[derive(Parser)]
#[command(name = "xtask", about = "samd5-boot bench tooling", version)]
struct Cli {
    #[command(subcommand)]
    command: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Build the demo: both application images, stamped, plus BOOT.
    Build {
        /// Build for the RS485 link rather than RTT.
        #[arg(long)]
        rs485: bool,
    },

    /// Stamp a manifest into a linked application image so BOOT verifies it.
    Stamp {
        input: PathBuf,
        output: PathBuf,
        #[arg(long)]
        version: u16,
    },

    /// Report what a part is configured as, without changing anything.
    Info {
        #[arg(long)]
        chip: String,
    },

    /// Write the BOOTPROT and region-lock fuses, then verify them after a
    /// reset.
    Provision {
        #[arg(long)]
        chip: String,
        /// BOOT region size. Must match the `bootprot-*` feature both
        /// firmware images were built with.
        #[arg(long, default_value = "32k", value_parser = parse_size)]
        boot_size: usize,
        /// Leave the BOOT regions unlocked rather than write protecting
        /// both copies from power-on.
        #[arg(long)]
        no_lock: bool,
        /// SmartEEPROM blocks (SBLK, 0..=10). Left alone when not given.
        #[arg(long)]
        sblk: Option<u8>,
        /// Write a saved user page back verbatim, ignoring the other
        /// options. `provision` saves one before it erases.
        #[arg(long)]
        restore: Option<PathBuf>,
        /// Print the register sequence instead of issuing it.
        #[arg(long)]
        dry_run: bool,
    },

    /// Set the update request through the debugger, so BOOT waits for an
    /// image instead of booting the application.
    RequestUpdate {
        #[arg(long)]
        chip: String,
        /// Byte offset of the boot record in backup RAM. The default is what
        /// the demo firmware uses.
        #[arg(long, default_value_t = demo_rig::STORE_OFFSET)]
        store_offset: usize,
        #[arg(long)]
        dry_run: bool,
    },

    /// Place a BOOT image at the head of both banks.
    Flash {
        #[arg(long)]
        chip: String,
        /// Defaults to the demo's BOOT image.
        #[arg(long)]
        boot: Option<PathBuf>,
        #[arg(long)]
        dry_run: bool,
    },

    /// Build, flash and run the rig self-test end to end.
    Test {
        #[arg(long)]
        chip: String,
        /// Drive the rig over RS485 on this serial device instead of over
        /// RTT. The firmware must have been built with `--features rs485`.
        #[arg(long, short)]
        port: Option<String>,
        /// Use the images and the BOOT already on the part.
        #[arg(long)]
        skip_build: bool,
        #[arg(long)]
        skip_flash: bool,
        /// Forward the target's log output (RTT only). The first thing to
        /// turn on when a step fails for no visible reason.
        #[arg(long)]
        log: bool,
    },

    /// Send one message to a device that is already running.
    Link {
        #[arg(long)]
        chip: String,
        /// Use RS485 on this serial device instead of RTT.
        #[arg(long, short)]
        port: Option<String>,
        /// Suppress the target's log output, which is forwarded by default
        /// here (RTT only).
        #[arg(long)]
        quiet: bool,
        #[command(subcommand)]
        op: LinkOp,
    },
}

#[derive(Subcommand)]
enum LinkOp {
    /// Liveness probe, answered by BOOT and by the application.
    Ping { text: Option<String> },
    /// Ask the application what it is running.
    State,
    /// Stream an image into the inactive bank.
    Update { image: PathBuf },
    /// The same with one body byte inverted, to drive the verify path.
    Bogus { image: PathBuf },
    /// Application flags an update request, then resets.
    Reboot,
    /// Application resets without flagging a request.
    Reset,
    /// Application condemns the running image and resets.
    Reject,
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
    match Cli::parse().command {
        Cmd::Build { rs485 } => {
            let built = image::build_demo(image::Transport::for_rs485(rs485))?;
            println!("\nbuilt:");
            for p in [&built.boot, &built.app, &built.app_noconfirm] {
                println!("  {}", p.display());
            }
            Ok(())
        }

        Cmd::Stamp {
            input,
            output,
            version,
        } => {
            tools_image::stamp(&input, &output, version)?;
            Ok(())
        }

        Cmd::Info { chip } => provision::info(&chip),

        Cmd::Provision {
            chip,
            boot_size,
            no_lock,
            sblk,
            restore,
            dry_run,
        } => provision::run(
            &chip,
            match restore {
                Some(path) => provision::Request::Restore(path),
                None => provision::Request::Fuses(provision::Fuses {
                    boot_size,
                    lock_boot: !no_lock,
                    see_sblk: sblk,
                }),
            },
            &image::repo_root(),
            mode(dry_run),
        ),

        Cmd::RequestUpdate {
            chip,
            store_offset,
            dry_run,
        } => provision::request_update(&chip, record_addr(store_offset), mode(dry_run)),

        Cmd::Flash {
            chip,
            boot,
            dry_run,
        } => {
            let boot = boot.unwrap_or_else(|| image::Artifacts::in_demo_dir().boot);
            flash::run(&chip, &boot, mode(dry_run))
        }

        Cmd::Test {
            chip,
            port,
            skip_build,
            skip_flash,
            log,
        } => self_test(&chip, port.as_deref(), skip_build, skip_flash, log),

        Cmd::Link {
            chip,
            port,
            quiet,
            op,
        } => {
            let mut link = open_link(&chip, port.as_deref(), !quiet)?;
            link_op(&mut link, op)
        }
    }
}

/// Open whichever transport was asked for.
///
/// RTT takes over the probe, so anything that needs its own debug session
/// (flashing, provisioning) has to finish before this is called.
fn open_link(chip: &str, port: Option<&str>, log: bool) -> Result<Link<Box<dyn Transport>>> {
    let transport: Box<dyn Transport> = match port {
        Some(path) => Box::new(Serial::open(path, link::serial::BAUD)?),
        None => Box::new(RttLink::attach(chip, log)?),
    };
    let link = Link::new(transport);
    eprintln!("link: {}", link.describe());
    Ok(link)
}

/// Where the rig's firmware keeps its boot record.
fn record_addr(store_offset: usize) -> u64 {
    (samd5_boot::consts::BKUPRAM_ADDR + store_offset) as u64
}

fn mode(dry_run: bool) -> Mode {
    if dry_run { Mode::DryRun } else { Mode::Run }
}

fn answers(link: &mut Link<Box<dyn Transport>>) -> bool {
    link.ping(*b"selftest", Duration::from_secs(3)).is_ok()
}

/// Send one message, with anything already buffered discarded first so the
/// reply cannot be confused with a frame from a previous exchange.
fn request(link: &mut Link<Box<dyn Transport>>, msg: &Message) -> Result<()> {
    link.flush_input()?;
    link.send(msg)
}

/// The ad-hoc operations, for poking at a device by hand.
fn link_op(link: &mut Link<Box<dyn Transport>>, op: LinkOp) -> Result<()> {
    let reply_timeout = Duration::from_secs(3);

    match op {
        LinkOp::Ping { text } => {
            let mut payload = [0u8; 8];
            let bytes = text.as_deref().unwrap_or("").as_bytes();
            let n = bytes.len().min(payload.len());
            payload[..n].copy_from_slice(&bytes[..n]);
            request(link, &Message::Ping(payload))?;
            match link.recv(Instant::now() + reply_timeout)? {
                Some(Message::Pong(echo)) if echo == payload => {
                    println!("<- Pong, echo matches");
                    Ok(())
                }
                Some(Message::Pong(echo)) => {
                    bail!("echo mismatch: sent {payload:02x?}, got {echo:02x?}")
                }
                Some(other) => bail!("unexpected {other:?}"),
                None => bail!("no Pong within {reply_timeout:?}"),
            }
        }

        LinkOp::State => {
            request(link, &Message::GetState)?;
            match link.recv(Instant::now() + reply_timeout)? {
                Some(Message::State {
                    app_version,
                    revert_reason,
                    confirmed,
                }) => {
                    println!(
                        "<- State {{ app_version: {app_version}, revert_reason: {revert_reason}, \
                         confirmed: {confirmed} }}"
                    );
                    Ok(())
                }
                Some(other) => bail!("unexpected {other:?}"),
                None => bail!("no State within {reply_timeout:?} (no application running?)"),
            }
        }

        LinkOp::Update { image } => send_image(link, &image, Body::AsBuilt),
        LinkOp::Bogus { image } => send_image(link, &image, Body::Corrupted),

        LinkOp::Reboot => {
            request(link, &Message::Update)?;
            println!("-> Update (application flags a request and resets)");
            Ok(())
        }
        LinkOp::Reset => {
            request(link, &Message::Reset)?;
            println!("-> Reset (BOOT takes its normal path)");
            Ok(())
        }
        LinkOp::Reject => {
            request(link, &Message::Reject)?;
            println!("-> Reject (application condemns itself and resets)");
            Ok(())
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Body {
    AsBuilt,
    Corrupted,
}

fn send_image(link: &mut Link<Box<dyn Transport>>, image: &Path, body: Body) -> Result<()> {
    let mut bytes =
        std::fs::read(image).with_context(|| format!("reading {}", image.display()))?;
    if body == Body::Corrupted {
        let at = image::corrupt_body(&mut bytes);
        println!("corrupted byte {at} of {}", bytes.len());
    }
    let len: u32 = bytes.len().try_into().context("image exceeds 4 GiB")?;
    request(link, &Message::BeginUpdate { len })?;
    let started = Instant::now();
    link.write_raw(&bytes)?;
    println!("   {len} bytes sent in {:.1}s", started.elapsed().as_secs_f32());
    match link.recv(Instant::now() + Duration::from_secs(60))? {
        Some(Message::UpdateResult(status)) => {
            println!("<- UpdateResult({status:?})");
            match status {
                Status::Ok => Ok(()),
                other => bail!("device rejected the image: {other:?}"),
            }
        }
        Some(other) => bail!("unexpected {other:?}"),
        None => {
            println!("no reply: the device swapped and rebooted (install succeeded)");
            Ok(())
        }
    }
}

/// Open a link with something alive on the far end, asking BOOT to wait
/// through the debugger if nothing is.
///
/// The usual reason nothing answers is that BOOT has booted an application
/// built for the other transport, which cannot be asked to step aside over a
/// link it does not speak. That shows up two different ways: over RTT there
/// is no control block to attach to, while a serial port opens perfectly
/// well and simply stays quiet. So the test is whether anything answers, not
/// whether the link opened.
fn live_link(chip: &str, port: Option<&str>, log: bool) -> Result<Link<Box<dyn Transport>>> {
    match open_link(chip, port, log) {
        Ok(mut link) => {
            if answers(&mut link) {
                return Ok(link);
            }
        }
        Err(e) if port.is_none() => eprintln!("{e:#}\n"),
        Err(e) => return Err(e),
    }
    println!("nothing answered on the link; asking BOOT to wait, through the debugger");
    // RTT holds the probe and `request_update` needs it: the link above is a
    // match binding, so it has already dropped by here.
    provision::request_update(chip, record_addr(demo_rig::STORE_OFFSET), Mode::Run)?;
    open_link(chip, port, log)
}

fn self_test(
    chip: &str,
    port: Option<&str>,
    skip_build: bool,
    skip_flash: bool,
    log: bool,
) -> Result<()> {
    let built = if skip_build {
        image::Artifacts::in_demo_dir()
    } else {
        // A `--port` means the rig is on RS485, so the firmware has to be.
        image::build_demo(image::Transport::for_rs485(port.is_some()))?
    };

    if !skip_flash {
        flash::run(chip, &built.boot, Mode::Run)?;
    }

    let mut link = live_link(chip, port, log)?;
    println!();
    match selftest::run(&mut link, &built.app, &built.app_noconfirm) {
        Ok(true) => Ok(()),
        Ok(false) => bail!("the self-test reported failures"),
        Err(e) => bail!(e),
    }
}
