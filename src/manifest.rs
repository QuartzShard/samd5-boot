//! Application image manifest: the fixed-layout descriptor BOOT reads to
//! verify an image before booting it. The application embeds one with
//! [`install_manifest!`](crate::install_manifest), which links it at
//! [`MANIFEST_OFFSET`] inside the app region; BOOT reads it there and checks
//! it in `Boot::verify`.
//!
//! The manifest is a frozen flash ABI: an installed BOOT must read manifests
//! from applications built years later, so new fields are append-only and
//! gated by [`FMT_VER`]. The `head` CRCs and the `body` are split so every
//! byte of the image except the two CRC fields themselves is covered by one
//! of them (see [`ManifestHeader`]).
//!
//! ## Signing reservation (weak-frozen)
//!
//! Nothing here signs or checks a signature. [`stamp`] writes
//! [`SigScheme::Unsigned`] and fills `sig` with `0xFF`, and `Boot::verify`
//! never reads [`ManifestBody::sig_scheme`], so an image that claims a
//! scheme is still accepted on its CRCs alone. The reservation is the
//! layout only: a signature added later would cover the image with its own
//! bytes excluded (image start to `sig`, then past `sig` to `image_len`)
//! under SHA-256, leaving the CRCs as a cheap pre-check.

/// Stored at [`MANIFEST_OFFSET`] inside the application image
#[derive(bytemuck::AnyBitPattern, bytemuck::NoUninit, Clone, Copy)]
#[repr(C)]
pub struct AppManifest {
    pub head: ManifestHeader,
    pub body: ManifestBody,
}

/// The two CRC32s that cover the image, split around themselves: together
/// they cover the whole image except these eight bytes. `crc32_vec_table`
/// covers the vector table below the manifest (image start up to `head`);
/// `crc32_image` covers `body` through the end of the image (`image_len`).
#[derive(bytemuck::AnyBitPattern, bytemuck::NoUninit, Clone, Copy)]
#[repr(C)]
pub struct ManifestHeader {
    pub crc32_vec_table: u32,
    pub crc32_image: u32,
}

/// The part of the manifest `crc32_image` covers. New fields are append
/// only: a BOOT built today reads the fields it knows from a manifest an
/// application writes later.
#[derive(bytemuck::AnyBitPattern, bytemuck::NoUninit, Clone, Copy)]
#[repr(C)]
pub struct ManifestBody {
    /// Selects the signing key among those baked into BOOT; 0 when unsigned
    pub pubkey_id: u64,
    pub magic: u32,
    /// Word-aligned; bounds `crc32_image` and is range-checked in
    /// `Boot::verify`
    pub image_len: u32,
    /// Anti-rollback counter (reserved for v2)
    pub version: u16,
    /// Layout this manifest was stamped with. BOOT refuses an image below
    /// its own [`FMT_VER`]; see the module docs for what that costs.
    pub fmt_version: u16,
    /// [`SigScheme`] as a raw byte; read it typed with
    /// [`ManifestBody::scheme`]
    pub sig_scheme: u8,
    /// Pads the body so it carries no implicit padding bytes. [`stamp`]
    /// zeroes it; nothing on-target checks it.
    pub reserved: [u8; 3],
    /// Reserved for a detached signature, sized for an ECDSA P-256 `r || s`.
    /// [`stamp`] fills it with `0xFF` and nothing reads it.
    pub sig: [u8; 64],
}

impl ManifestBody {
    /// Typed view of [`sig_scheme`](Self::sig_scheme)
    pub const fn scheme(&self) -> Option<SigScheme> {
        SigScheme::from_u8(self.sig_scheme)
    }
}

/// Signature algorithm over the image, the typed form of the raw
/// [`ManifestBody::sig_scheme`] byte. `Unsigned` leaves `sig` all `0xFF`;
/// `EcdsaP256Sha256` is a big-endian `r || s`, reserved for v2.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum SigScheme {
    Unsigned = 0,
    EcdsaP256Sha256 = 1,
}

impl SigScheme {
    /// Decode a stored [`ManifestBody::sig_scheme`] byte
    ///
    /// `None` is a scheme this build does not know. Nothing decodes the byte
    /// today: `Boot::verify` does not look at it, so an unknown scheme is
    /// not by itself a reason an image is refused.
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Unsigned),
            1 => Some(Self::EcdsaP256Sha256),
            _ => None,
        }
    }
}

/// Manifest magic ([`ManifestBody::magic`]); the first check in verification
pub const MAGIC: u32 = 0xa55af00b;
/// Lowest manifest layout a bootloader built from this source accepts
/// ([`ManifestBody::fmt_version`])
pub const FMT_VER: u16 = 1;

// Weak-frozen layout: pin the size and the signed-region split point so a
// field reshuffle cannot silently move them. `sig` at body offset 24 keeps
// `AppManifest` at 96 bytes with no implicit padding.
const _: () = assert!(size_of::<AppManifest>() == 96);
const _: () = assert!(core::mem::offset_of!(ManifestBody, sig) == 24);

impl AppManifest {
    /// An unsigned, unstamped manifest to reserve the slot with
    /// [`install_manifest!`](crate::install_manifest). [`stamp`] fills in
    /// the length, the version and the CRCs after linking;
    /// `samd5-boot-tools` is what calls it.
    pub const fn placeholder() -> Self {
        Self {
            head: ManifestHeader {
                crc32_vec_table: 0,
                crc32_image: 0,
            },
            body: ManifestBody {
                pubkey_id: 0,
                magic: MAGIC,
                image_len: 0,
                version: 0,
                fmt_version: FMT_VER,
                sig_scheme: SigScheme::Unsigned as u8,
                reserved: [0; 3],
                sig: [0xFF; 64],
            },
        }
    }
}

/// Place the app's manifest at the fixed in-image slot BOOT reads it from
/// ([`consts::MANIFEST_OFFSET`](crate::consts::MANIFEST_OFFSET)). Call
/// once at item level in the app binary; requires linking with
/// `samd5_boot_app.x`, which anchors the section and reserves the slot.
#[macro_export]
macro_rules! install_manifest {
    ($manifest:expr) => {
        #[unsafe(link_section = ".samd5_boot_manifest")]
        #[used]
        static SAMD5_BOOT_MANIFEST: $crate::manifest::AppManifest = $manifest;
    };
}

// ── Post-link stamping ──────────────────────────────────────────────────
//
// The offsets a stamper writes at are derived from the struct above rather
// than restated, so a field reshuffle moves both the reader and the writer
// together. `check_slot` recomputes the same two ranges on-target.

use crate::{consts::MANIFEST_OFFSET, crc32::crc32};
use core::mem::offset_of;

const OFF_CRC_VEC: usize = MANIFEST_OFFSET + offset_of!(AppManifest, head.crc32_vec_table);
const OFF_CRC_IMG: usize = MANIFEST_OFFSET + offset_of!(AppManifest, head.crc32_image);
/// Start of the `crc32_image` range: the body, i.e. everything past the two
/// CRC fields that cannot cover themselves
pub const OFF_BODY: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body);
const OFF_PUBKEY: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body.pubkey_id);
const OFF_MAGIC: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body.magic);
const OFF_IMAGE_LEN: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body.image_len);
const OFF_VERSION: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body.version);
const OFF_FMT_VERSION: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body.fmt_version);
const OFF_SIG_SCHEME: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body.sig_scheme);
const OFF_RESERVED: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body.reserved);
const OFF_SIG: usize = MANIFEST_OFFSET + offset_of!(AppManifest, body.sig);

/// Smallest image a manifest fits in: the slot must be wholly present.
/// This is the floor the bootloader applies to `image_len` when it verifies
/// a slot; the ceiling is a runtime value (chip density and the live
/// SmartEEPROM reserve) and is checked there.
pub const MIN_IMAGE_LEN: usize = MANIFEST_OFFSET + size_of::<AppManifest>();

/// Why [`stamp`] could not stamp an image
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StampError {
    /// The manifest slot does not fit; `len` is what was offered
    TooShort { len: usize, need: usize },
    /// `crc32_image` runs to `image_len`, which the DSU walks in words
    Unaligned { len: usize },
}

/// What was written, for a caller that wants to report or re-check it
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Stamped {
    pub image_len: u32,
    pub version: u16,
    pub crc32_vec_table: u32,
    pub crc32_image: u32,
}

/// Stamp a valid unsigned v1 manifest into a linked image, in place
///
/// `image` must be an application image linked with `samd5_boot_app.x`: the
/// offsets are fixed, so stamping an image with no manifest slot overwrites
/// whatever is at [`MANIFEST_OFFSET`] instead.
///
/// `image.len()` becomes `image_len`, so the caller pads the image to its
/// final length first; a length that is not a multiple of 4 is rejected.
/// Order matters: every body field is written before `crc32_image` is taken
/// over the body range, and the head CRCs go last because nothing may change
/// underneath them.
///
/// # Errors
///
/// * Returns [`StampError::TooShort`] if `image` is smaller than
///   [`MIN_IMAGE_LEN`], so the manifest slot is not wholly present.
/// * Returns [`StampError::Unaligned`] if `image.len()` is not a multiple
///   of 4.
pub fn stamp(image: &mut [u8], version: u16) -> Result<Stamped, StampError> {
    let len = image.len();
    if len < MIN_IMAGE_LEN {
        return Err(StampError::TooShort {
            len,
            need: MIN_IMAGE_LEN,
        });
    }
    if !len.is_multiple_of(4) {
        return Err(StampError::Unaligned { len });
    }

    write_u64(image, OFF_PUBKEY, 0);
    write_u32(image, OFF_MAGIC, MAGIC);
    write_u32(image, OFF_IMAGE_LEN, len as u32);
    write_u16(image, OFF_VERSION, version);
    write_u16(image, OFF_FMT_VERSION, FMT_VER);
    // An unstamped slot is 0xFF, and 0xFF would decode on-target as an
    // unknown scheme rather than as unsigned.
    image[OFF_SIG_SCHEME] = SigScheme::Unsigned as u8;
    image[OFF_RESERVED..OFF_RESERVED + 3].fill(0);
    image[OFF_SIG..OFF_SIG + 64].fill(0xFF);

    let crc32_vec_table = crc32(&image[..MANIFEST_OFFSET]);
    let crc32_image = crc32(&image[OFF_BODY..len]);
    write_u32(image, OFF_CRC_VEC, crc32_vec_table);
    write_u32(image, OFF_CRC_IMG, crc32_image);

    Ok(Stamped {
        image_len: len as u32,
        version,
        crc32_vec_table,
        crc32_image,
    })
}

/// The version a stamped image carries, or `None` if it holds no manifest
/// with a recognised [`MAGIC`] and an [`FMT_VER`] this build can read
///
/// This is an identity check, not a verification: `image_len` and the two
/// CRCs are not looked at, so an image that reads a version here can still
/// fail on-target.
pub fn read_version(image: &[u8]) -> Option<u16> {
    if image.len() < MIN_IMAGE_LEN {
        return None;
    }
    let manifest: AppManifest =
        bytemuck::pod_read_unaligned(&image[MANIFEST_OFFSET..][..size_of::<AppManifest>()]);
    (manifest.body.magic == MAGIC && manifest.body.fmt_version >= FMT_VER)
        .then_some(manifest.body.version)
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

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec;

    /// The flash ABI as byte offsets. An installed BOOT reads manifests from
    /// applications built years later, so these are frozen: a change here is
    /// a change every fielded device disagrees with.
    #[test]
    fn offsets_are_pinned() {
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
        assert_eq!(size_of::<AppManifest>(), 0x60);
        assert_eq!(MIN_IMAGE_LEN, 0x460);
    }

    #[test]
    fn stamps_what_check_slot_reads() {
        let mut image = vec![0xFFu8; MIN_IMAGE_LEN + 64];
        // A recognisable vector table, so crc32_vec_table is not a constant.
        for (i, b) in image[..0x100].iter_mut().enumerate() {
            *b = i as u8;
        }
        let stamped = stamp(&mut image, 7).unwrap();

        assert_eq!(stamped.image_len as usize, image.len());
        assert_eq!(read_version(&image), Some(7));
        assert_eq!(
            u32::from_le_bytes(image[OFF_MAGIC..OFF_MAGIC + 4].try_into().unwrap()),
            MAGIC
        );
        assert_eq!(image[OFF_SIG_SCHEME], SigScheme::Unsigned as u8);
        // The head CRCs are the only bytes no CRC covers.
        assert_eq!(stamped.crc32_vec_table, crc32(&image[..MANIFEST_OFFSET]));
        assert_eq!(stamped.crc32_image, crc32(&image[OFF_BODY..image.len()]));
    }

    #[test]
    fn refuses_images_it_cannot_stamp() {
        assert!(matches!(
            stamp(&mut vec![0xFF; MIN_IMAGE_LEN - 4], 1),
            Err(StampError::TooShort { .. })
        ));
        assert!(matches!(
            stamp(&mut vec![0xFF; MIN_IMAGE_LEN + 1], 1),
            Err(StampError::Unaligned { .. })
        ));
    }

    /// An unstamped image must not read as a valid one.
    #[test]
    fn unstamped_has_no_version() {
        assert_eq!(read_version(&vec![0xFF; MIN_IMAGE_LEN]), None);
        assert_eq!(read_version(&[0u8; 16]), None);
    }
}
