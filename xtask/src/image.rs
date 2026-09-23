//! Building and stamping the demo images.
//!
//! The manifest stamp comes from `samd5_boot::manifest::stamp`, the same
//! code and the same offsets BOOT reads back, so an image that stamps here
//! is one `check_slot` accepts by construction.

use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{Context, Result, bail};
use samd5_boot::manifest;

const TRIPLE: &str = "thumbv7em-none-eabihf";

/// The two application builds the self-test needs. They differ only in
/// whether the image confirms itself, and carry different versions so the
/// host can tell from the wire which one a device came back running.
pub const APP_VERSION: u16 = 1;
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

pub fn build_demo(transport: Transport) -> Result<Artifacts> {
    let demo = demo_dir();
    let out = Artifacts::in_demo_dir();

    println!("[1/3] app (confirming build)");
    cargo(&demo.join("app"), &transport.args(&[]))?;
    let app_raw = objcopy(&demo.join("app"), "app", &demo.join("app-raw.bin"))?;
    stamp(&app_raw, &out.app, APP_VERSION)?;

    println!("[2/3] app (no-confirm build)");
    cargo(&demo.join("app"), &transport.args(&["noconfirm"]))?;
    let nc_raw = objcopy(&demo.join("app"), "app", &demo.join("app-nc-raw.bin"))?;
    stamp(&nc_raw, &out.app_noconfirm, APP_NOCONFIRM_VERSION)?;

    println!("[3/3] boot");
    cargo(&demo.join("boot"), &transport.args(&[]))?;
    objcopy(&demo.join("boot"), "demo-boot", &out.boot)?;

    Ok(out)
}

/// Stamp a linked image so BOOT will verify it, reporting what went in.
pub fn stamp(input: &Path, output: &Path, version: u16) -> Result<PathBuf> {
    let mut image = std::fs::read(input).with_context(|| format!("reading {}", input.display()))?;
    // `crc32_image` runs to `image_len`, and the DSU walks whole words.
    while !image.len().is_multiple_of(4) {
        image.push(0xFF);
    }
    let stamped = match manifest::stamp(&mut image, version) {
        Ok(s) => s,
        Err(manifest::StampError::TooShort { len, need }) => bail!(
            "{} is {len} bytes; a manifest needs at least {need}. Is the manifest slot \
             reserved (install_manifest! plus samd5_boot_app.x)?",
            input.display()
        ),
        Err(manifest::StampError::Unaligned { len }) => {
            bail!("{} is {len} bytes, which is not a whole number of words", input.display())
        }
    };
    std::fs::write(output, &image).with_context(|| format!("writing {}", output.display()))?;
    println!(
        "  stamped {} ({} bytes): version={} crc32_vec_table={:#010x} crc32_image={:#010x}",
        output.display(),
        stamped.image_len,
        stamped.version,
        stamped.crc32_vec_table,
        stamped.crc32_image
    );
    Ok(output.to_path_buf())
}

/// Flip a byte in the middle of the body: past the vector table, inside
/// the range the device's CRC covers. Returns the offset it flipped.
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
