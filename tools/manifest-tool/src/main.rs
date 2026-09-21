//! manifest-tool: host-side post-linker for samd5-boot application images.
//!
//! Takes a raw linked app image (`.bin`, objcopy'd, with the manifest slot
//! reserved but its CRC/magic fields unset) and stamps a valid v1 (unsigned)
//! `AppManifest` into it so the samd5-boot bootloader's `check_slot` accepts
//! the image.
//!
//! # Usage
//!
//! ```text
//! manifest-tool <input.bin> <output.bin> --version <N> [--pad]
//! ```
//!
//! - `<input.bin>`  raw linked image with the reserved manifest slot present.
//! - `<output.bin>` where the patched image is written (may equal input).
//! - `--version N`  app version stamped into `body.version` (u16, dec or 0x hex).
//! - `--pad`        0xFF-pad the image up to a 4-byte boundary rather than
//!   erroring when its length is not a multiple of 4.
//!
//! # What it stamps
//!
//! At byte [`MANIFEST_OFFSET`] (0x400) the manifest is laid out `#[repr(C)]`
//! exactly as `samd5-boot`'s `manifest.rs` defines it. The tool writes:
//! `magic = MAGIC`, `fmt_version = FMT_VER`, `version` (CLI), `image_len` =
//! actual image length, `pubkey_id = 0`, `sig_scheme = 0` (unsigned) with its
//! reserve zeroed, and `sig = [0xFF; 64]`, then the two head CRCs over the
//! exact ranges `check_slot` verifies. Signing (v2) fills `sig_scheme`/`sig`.
//!
//! # CRC convention (must match the SAMD5x DSU)
//!
//! Standard zlib CRC-32, a.k.a. CRC-32/ISO-HDLC:
//!   width 32, poly 0x04C11DB7 (reflected form 0xEDB88320), init 0xFFFFFFFF,
//!   refin = true, refout = true, xorout = 0xFFFFFFFF.
//! Canonical check: crc32(b"123456789") == 0xCBF43926 (pinned in tests).
//!
//! The DSU walks memory in ascending-address, byte-at-a-time order; run over
//! the identical file bytes the result is identical. Two ranges are covered,
//! splitting around the 8-byte head so those CRC fields are the only bytes
//! not under a CRC:
//!
//! - `crc32_vec_table` over image bytes `[0, MANIFEST_OFFSET)`
//! - `crc32_image` over image bytes `[MANIFEST_OFFSET + offset_of!(body), image_len)`
//!
//! (`offset_of!(AppManifest, body)` is 8, so the second range starts at 0x408.)

use std::mem::offset_of;
use std::process::ExitCode;

mod crc32;

// ── Manifest layout, mirrored from samd5-boot `src/manifest.rs` ─────────────
//
// These structs exist only to derive #[repr(C)] byte offsets; they are never
// instantiated. Keep them field-for-field identical to the embedded crate:
// the byte offsets computed below (and pinned in tests) are the contract.

pub const MAGIC: u32 = 0xa55a_f00b;
pub const FMT_VER: u16 = 1;
/// Manifest byte offset from the image base (samd5-boot `consts::MANIFEST_OFFSET`).
pub const MANIFEST_OFFSET: usize = 0x400;

#[repr(C)]
struct ManifestHeader {
    crc32_vec_table: u32,
    crc32_image: u32,
}

#[repr(C)]
struct ManifestBody {
    pubkey_id: u64,
    magic: u32,
    image_len: u32,
    version: u16,
    fmt_version: u16,
    sig_scheme: u8,
    reserved: [u8; 3],
    sig: [u8; 64],
}

#[repr(C)]
struct AppManifest {
    head: ManifestHeader,
    body: ManifestBody,
}

// Absolute byte offsets of each field within the image. `offset_of!` follows
// the same #[repr(C)] rules the bootloader reads the struct back with.
const OFF_CRC_VEC: usize = MANIFEST_OFFSET + offset_of!(AppManifest, head.crc32_vec_table);
const OFF_CRC_IMG: usize = MANIFEST_OFFSET + offset_of!(AppManifest, head.crc32_image);
const OFF_BODY: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body);
const OFF_PUBKEY: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body.pubkey_id);
const OFF_MAGIC: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body.magic);
const OFF_IMAGE_LEN: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body.image_len);
const OFF_VERSION: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body.version);
const OFF_FMT_VERSION: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body.fmt_version);
const OFF_SIG_SCHEME: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body.sig_scheme);
const OFF_RESERVED: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body.reserved);
const OFF_SIG: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body.sig);

/// Total manifest size, i.e. the smallest image the slot fits in past 0x400.
const MANIFEST_SIZE: usize = size_of::<AppManifest>();
/// Minimum valid image length: the manifest must fully exist. This mirrors the
/// floor `check_slot` enforces on `image_len` (the ceiling is a runtime value
/// depending on chip density and SmartEEPROM reserve, so it is not checked here).
const MIN_IMAGE_LEN: usize = MANIFEST_OFFSET + MANIFEST_SIZE;

/// Where the body-CRC range begins (0x408). Named to match `check_slot`.
const BODY_OFFSET: usize = OFF_BODY;

#[derive(Debug)]
pub struct PatchError(String);

impl std::fmt::Display for PatchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for PatchError {}

fn err<T>(msg: impl Into<String>) -> Result<T, PatchError> {
    Err(PatchError(msg.into()))
}

/// Stamp a valid unsigned-v1 manifest into `image` in place.
///
/// `image.len()` is taken as `image_len`; it must be 4-byte aligned (the caller
/// pads first if `--pad` was given) and at least [`MIN_IMAGE_LEN`]. Order
/// matters: every body field is written before `crc32_image` is computed over
/// the body range.
pub fn patch(image: &mut [u8], version: u16) -> Result<(), PatchError> {
    let len = image.len();
    if len < MIN_IMAGE_LEN {
        return err(format!(
            "image is {len} bytes; needs at least {MIN_IMAGE_LEN} (0x{MIN_IMAGE_LEN:x}) to hold the manifest at 0x{MANIFEST_OFFSET:x}"
        ));
    }
    if !len.is_multiple_of(4) {
        return err(format!(
            "image length {len} is not a multiple of 4 (pass --pad to 0xFF-pad up to a word boundary)"
        ));
    }

    // Body fields first: crc32_image covers [BODY_OFFSET, image_len).
    write_u64(image, OFF_PUBKEY, 0);
    write_u32(image, OFF_MAGIC, MAGIC);
    write_u32(image, OFF_IMAGE_LEN, len as u32);
    write_u16(image, OFF_VERSION, version);
    write_u16(image, OFF_FMT_VERSION, FMT_VER);
    // Unsigned scheme (0) and zeroed reserve: the incoming slot is 0xFF, and
    // sig_scheme = 0xFF would read on-target as an unknown scheme.
    image[OFF_SIG_SCHEME] = 0;
    image[OFF_RESERVED..OFF_RESERVED + 3].fill(0);
    image[OFF_SIG..OFF_SIG + 64].fill(0xFF);

    // Head CRCs last. Ranges are exactly what check_slot recomputes on-target.
    let crc_vec = crc32::crc32(&image[0..MANIFEST_OFFSET]);
    let crc_img = crc32::crc32(&image[BODY_OFFSET..len]);
    write_u32(image, OFF_CRC_VEC, crc_vec);
    write_u32(image, OFF_CRC_IMG, crc_img);
    Ok(())
}

fn write_u16(buf: &mut [u8], off: usize, v: u16) {
    buf[off..off + 2].copy_from_slice(&v.to_le_bytes());
}
fn write_u32(buf: &mut [u8], off: usize, v: u32) {
    buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
}
fn write_u64(buf: &mut [u8], off: usize, v: u64) {
    buf[off..off + 8].copy_from_slice(&v.to_le_bytes());
}

// ── CLI ─────────────────────────────────────────────────────────────────────

const USAGE: &str =
    "usage: manifest-tool <input.bin> <output.bin> --version <N> [--pad]\n\
     \n\
     \x20 <input.bin>   raw linked image with the reserved manifest slot\n\
     \x20 <output.bin>  patched image output (may equal input)\n\
     \x20 --version N   app version stamped into the manifest (u16; dec or 0x hex)\n\
     \x20 --pad         0xFF-pad the image up to a 4-byte boundary before stamping";

struct Args {
    input: String,
    output: String,
    version: u16,
    pad: bool,
}

fn parse_args(argv: &[String]) -> Result<Args, PatchError> {
    let mut positional: Vec<String> = Vec::new();
    let mut version: Option<u16> = None;
    let mut pad = false;

    let mut i = 0;
    while i < argv.len() {
        let a = &argv[i];
        match a.as_str() {
            "--version" => {
                i += 1;
                let raw = argv
                    .get(i)
                    .ok_or_else(|| PatchError("--version needs a value".into()))?;
                version = Some(parse_u16(raw)?);
            }
            "--pad" => pad = true,
            "-h" | "--help" => return err("help"),
            s if s.starts_with("--") => return err(format!("unknown flag: {s}")),
            _ => positional.push(a.clone()),
        }
        i += 1;
    }

    if positional.len() != 2 {
        return err(format!(
            "expected exactly 2 positional args (input, output), got {}",
            positional.len()
        ));
    }
    let version = version.ok_or_else(|| PatchError("--version is required".into()))?;
    Ok(Args {
        input: positional[0].clone(),
        output: positional[1].clone(),
        version,
        pad,
    })
}

fn parse_u16(raw: &str) -> Result<u16, PatchError> {
    let parsed = if let Some(hex) = raw.strip_prefix("0x").or_else(|| raw.strip_prefix("0X")) {
        u16::from_str_radix(hex, 16)
    } else {
        raw.parse::<u16>()
    };
    parsed.map_err(|_| PatchError(format!("invalid --version value: {raw:?} (expected 0..=65535)")))
}

fn run(argv: &[String]) -> Result<(), PatchError> {
    let args = match parse_args(argv) {
        Ok(a) => a,
        Err(e) if e.0 == "help" => {
            println!("{USAGE}");
            return Ok(());
        }
        Err(e) => return Err(e),
    };

    let mut image = std::fs::read(&args.input)
        .map_err(|e| PatchError(format!("reading {}: {e}", args.input)))?;

    if args.pad {
        let pad = (4 - image.len() % 4) % 4;
        image.resize(image.len() + pad, 0xFF);
    }

    patch(&mut image, args.version)?;

    std::fs::write(&args.output, &image)
        .map_err(|e| PatchError(format!("writing {}: {e}", args.output)))?;

    let crc_vec = u32::from_le_bytes(image[OFF_CRC_VEC..OFF_CRC_VEC + 4].try_into().unwrap());
    let crc_img = u32::from_le_bytes(image[OFF_CRC_IMG..OFF_CRC_IMG + 4].try_into().unwrap());
    eprintln!(
        "stamped {} ({} bytes): version={} crc32_vec_table=0x{crc_vec:08x} crc32_image=0x{crc_img:08x}",
        args.output,
        image.len(),
        args.version,
    );
    Ok(())
}

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    match run(&argv) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("manifest-tool: error: {e}");
            eprintln!("{USAGE}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Field offsets are the on-target ABI contract: pin every one. If the
    // mirrored struct ever drifts from samd5-boot's manifest.rs, this trips.
    #[test]
    fn manifest_offsets_are_pinned() {
        assert_eq!(OFF_CRC_VEC, 0x400);
        assert_eq!(OFF_CRC_IMG, 0x404);
        assert_eq!(OFF_BODY, 0x408);
        assert_eq!(OFF_PUBKEY, 0x408);
        assert_eq!(OFF_MAGIC, 0x410);
        assert_eq!(OFF_IMAGE_LEN, 0x414);
        assert_eq!(OFF_VERSION, 0x418);
        assert_eq!(OFF_FMT_VERSION, 0x41a);
        assert_eq!(OFF_SIG_SCHEME, 0x41c);
        assert_eq!(OFF_RESERVED, 0x41d);
        assert_eq!(OFF_SIG, 0x420);
        // sig_scheme (1) + reserved (3) place sig at 0x420; sig is 64 bytes,
        // so the manifest ends at 0x460 with no implicit padding.
        assert_eq!(MANIFEST_SIZE, 0x60);
        assert_eq!(MIN_IMAGE_LEN, 0x460);
        // The body CRC range starts 8 bytes past the manifest, matching the
        // CHECKLIST's `[MANIFEST_OFFSET+8, image_len)`.
        assert_eq!(BODY_OFFSET - MANIFEST_OFFSET, 8);
    }

    // Build a deterministic image, stamp it, then re-verify BOTH ranges the
    // exact way the bootloader's check_slot does. This is the round-trip that
    // proves the tool and the on-target verifier agree.
    #[test]
    fn round_trip_matches_bootloader() {
        let len = 0x500usize; // 1280 bytes, word aligned, > MIN_IMAGE_LEN
        let mut image: Vec<u8> = (0..len).map(|i| (i * 7 + 3) as u8).collect();
        // Emulate an unstamped reserved slot (objcopy leaves it 0xFF).
        image[MANIFEST_OFFSET..MANIFEST_OFFSET + MANIFEST_SIZE].fill(0xFF);

        let version = 0x0203u16;
        patch(&mut image, version).unwrap();

        // Body fields landed where the ABI says.
        assert_eq!(read_u32(&image, OFF_MAGIC), MAGIC);
        assert_eq!(read_u16(&image, OFF_FMT_VERSION), FMT_VER);
        assert_eq!(read_u16(&image, OFF_VERSION), version);
        assert_eq!(read_u32(&image, OFF_IMAGE_LEN), len as u32);
        assert_eq!(read_u64(&image, OFF_PUBKEY), 0);
        assert!(image[OFF_SIG..OFF_SIG + 64].iter().all(|&b| b == 0xFF));

        // Recompute exactly as check_slot: head over [0, 0x400), body over
        // [MANIFEST_OFFSET + offset_of!(body), image_len).
        let expect_vec = crc32::crc32(&image[0..MANIFEST_OFFSET]);
        let body_start = MANIFEST_OFFSET + 8;
        let expect_img = crc32::crc32(&image[body_start..len]);

        assert_eq!(read_u32(&image, OFF_CRC_VEC), expect_vec);
        assert_eq!(read_u32(&image, OFF_CRC_IMG), expect_img);

        // The 8 head bytes are the ONLY bytes not under a CRC: flipping any
        // byte outside them must change one of the two recomputed CRCs.
        for probe in [0usize, MANIFEST_OFFSET - 1, body_start, len - 1] {
            let mut m = image.clone();
            m[probe] ^= 0xFF;
            let v = crc32::crc32(&m[0..MANIFEST_OFFSET]);
            let g = crc32::crc32(&m[body_start..len]);
            assert!(
                v != expect_vec || g != expect_img,
                "byte {probe:#x} is outside both CRC ranges but should be covered"
            );
        }
    }

    #[test]
    fn image_len_stamped_after_pad_is_word_aligned_length() {
        // A 0x461-byte image (one past the floor, not word aligned) must be
        // rejected without --pad and stamped with the padded length with it.
        let mut unpadded = vec![0xAAu8; 0x461];
        unpadded[MANIFEST_OFFSET..MANIFEST_OFFSET + MANIFEST_SIZE].fill(0xFF);
        assert!(patch(&mut unpadded.clone(), 1).is_err());

        let pad = (4 - unpadded.len() % 4) % 4;
        unpadded.resize(unpadded.len() + pad, 0xFF);
        assert_eq!(unpadded.len(), 0x464);
        patch(&mut unpadded, 1).unwrap();
        assert_eq!(read_u32(&unpadded, OFF_IMAGE_LEN), 0x464);
    }

    #[test]
    fn rejects_too_small() {
        let mut tiny = vec![0u8; MIN_IMAGE_LEN - 4];
        assert!(patch(&mut tiny, 1).is_err());
    }

    #[test]
    fn cli_version_parsing() {
        assert_eq!(parse_u16("7").unwrap(), 7);
        assert_eq!(parse_u16("0x0203").unwrap(), 0x0203);
        assert_eq!(parse_u16("65535").unwrap(), 0xFFFF);
        assert!(parse_u16("65536").is_err());
        assert!(parse_u16("nope").is_err());
    }

    fn read_u16(buf: &[u8], off: usize) -> u16 {
        u16::from_le_bytes(buf[off..off + 2].try_into().unwrap())
    }
    fn read_u32(buf: &[u8], off: usize) -> u32 {
        u32::from_le_bytes(buf[off..off + 4].try_into().unwrap())
    }
    fn read_u64(buf: &[u8], off: usize) -> u64 {
        u64::from_le_bytes(buf[off..off + 8].try_into().unwrap())
    }
}
