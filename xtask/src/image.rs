//! Building and stamping the demo images. [`build_demo`] is the entry
//! point and [`Artifacts::in_demo_dir`] names what it writes.
//!
//! Stamping goes through `samd5_boot_tools::image::stamp` onto
//! `samd5_boot::manifest::stamp`, the same code and the same offsets BOOT
//! reads back, so a stamped image carries the CRCs `Boot::verify`
//! recomputes. Verification also bounds `image_len` by the app region,
//! which stamping cannot check.

use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{Context, Result, bail};
use samd5_boot_tools::image::stamp;

const TRIPLE: &str = "thumbv7em-none-eabihf";

/// Stamped into the manifest of the confirming build.
///
/// Must match the `APP_VERSION` const in `examples/update-rig/app`, which
/// is what the image reports on the wire; the self-test compares the two.
pub const APP_VERSION: u16 = 1;
/// Stamped into the manifest of the `noconfirm` build, which never confirms
/// itself. Must match that crate's `APP_VERSION` under `noconfirm`.
pub const APP_NOCONFIRM_VERSION: u16 = 2;

pub struct Artifacts {
    pub boot: PathBuf,
    pub app: PathBuf,
    pub app_noconfirm: PathBuf,
}

impl Artifacts {
    /// Where `build_demo` puts them, for callers that skipped the build.
    pub fn in_demo_dir() -> Self {
        let demo = demo_dir();
        Self {
            boot: demo.join("demo-boot.bin"),
            app: demo.join("app.bin"),
            app_noconfirm: demo.join("app-noconfirm.bin"),
        }
    }
}

pub fn demo_dir() -> PathBuf {
    repo_root().join("examples/update-rig")
}

/// The repository root, from this crate's manifest rather than the working
/// directory, so `cargo xtask` works from anywhere in the tree.
pub fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("xtask/ has a parent")
        .to_path_buf()
}

/// Which link the firmware is built to speak. RTT needs only the probe, so
/// it is the default; RS485 additionally needs a transceiver, and both
/// images plus the host must agree.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Transport {
    Rtt,
    Rs485,
}

impl Transport {
    pub fn for_rs485(rs485: bool) -> Self {
        if rs485 { Self::Rs485 } else { Self::Rtt }
    }

    /// Cargo arguments selecting this transport, plus any extra features.
    fn args(self, extra: &[&str]) -> Vec<String> {
        let mut args: Vec<String> = ["build", "--release"].iter().map(|s| s.to_string()).collect();
        let mut features: Vec<&str> = extra.to_vec();
        if self == Transport::Rs485 {
            args.push("--no-default-features".into());
            features.push("rs485");
        }
        if !features.is_empty() {
            args.push("--features".into());
            args.push(features.join(","));
        }
        args
    }
}

/// Build both application images and BOOT for `transport`, stamp the two
/// applications, and return where they landed.
///
/// Also leaves the unstamped `app-raw.bin` and `app-nc-raw.bin` in the demo
/// directory.
pub fn build_demo(transport: Transport) -> Result<Artifacts> {
    let demo = demo_dir();
    let out = Artifacts::in_demo_dir();

    println!("[1/3] app (confirming build)");
    cargo(&demo.join("app"), &transport.args(&[]))?;
    let app_raw = objcopy(&demo.join("app"), "app", &demo.join("app-raw.bin"))?;
    report(stamp(&app_raw, &out.app, APP_VERSION)?, &out.app);

    println!("[2/3] app (no-confirm build)");
    cargo(&demo.join("app"), &transport.args(&["noconfirm"]))?;
    let nc_raw = objcopy(&demo.join("app"), "app", &demo.join("app-nc-raw.bin"))?;
    report(
        stamp(&nc_raw, &out.app_noconfirm, APP_NOCONFIRM_VERSION)?,
        &out.app_noconfirm,
    );

    println!("[3/3] boot");
    cargo(&demo.join("boot"), &transport.args(&[]))?;
    objcopy(&demo.join("boot"), "demo-boot", &out.boot)?;

    Ok(out)
}

/// Invert one byte in the middle of the image and return its offset.
///
/// The midpoint of a demo image is past the vector table and inside the
/// range `crc32_image` covers, so the device fails it in verification.
pub fn corrupt_body(image: &mut [u8]) -> usize {
    let at = image.len() / 2;
    image[at] ^= 0xFF;
    at
}

fn cargo(dir: &Path, args: &[String]) -> Result<()> {
    let status = Command::new(env!("CARGO"))
        .current_dir(dir)
        .args(args)
        .status()
        .with_context(|| format!("running cargo in {}", dir.display()))?;
    if !status.success() {
        bail!("cargo {} failed in {}", args.join(" "), dir.display());
    }
    Ok(())
}

fn objcopy(crate_dir: &Path, bin: &str, out: &Path) -> Result<PathBuf> {
    let elf = crate_dir.join("target").join(TRIPLE).join("release").join(bin);
    let tool = std::env::var("OBJCOPY").unwrap_or_else(|_| "rust-objcopy".into());
    let status = Command::new(&tool)
        .args(["-O", "binary"])
        .arg(&elf)
        .arg(out)
        .status()
        .with_context(|| {
            format!("running {tool}; install it with `cargo install cargo-binutils` and \
                     `rustup component add llvm-tools`, or set OBJCOPY")
        })?;
    if !status.success() {
        bail!("{tool} failed on {}", elf.display());
    }
    Ok(out.to_path_buf())
}

fn report(stamped: samd5_boot::manifest::Stamped, path: &Path) {
    println!(
        "  stamped {} ({} bytes): version={} crc32_vec_table={:#010x} crc32_image={:#010x}",
        path.display(),
        stamped.image_len,
        stamped.version,
        stamped.crc32_vec_table,
        stamped.crc32_image
    );
}
