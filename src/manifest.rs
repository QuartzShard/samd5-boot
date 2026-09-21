//! Application image manifest: the fixed-layout descriptor BOOT reads to
//! verify an image before booting it. The application embeds one with
//! [`install_manifest!`](crate::install_manifest), which links it at
//! [`MANIFEST_OFFSET`](crate::consts::MANIFEST_OFFSET) inside the app region;
//! BOOT reads it there and checks it in `Boot::verify`.
//!
//! The manifest is a frozen flash ABI: an installed BOOT must read manifests
//! from applications built years later, so new fields are append-only and
//! gated by [`FMT_VER`]. The `head` CRCs and the `body` are split so every
//! byte of the image except the two CRC fields themselves is covered by one
//! of them (see [`ManifestHeader`]).
//!
//! ## Signing reservation (weak-frozen)
//!
//! The layout carries everything image signing needs so it can arrive
//! without an ABI break: [`ManifestBody::sig`] (64 bytes, sized for an
//! ECDSA P-256 `r || s`), [`ManifestBody::pubkey_id`] (which trusted key
//! signed it, selecting among the public keys baked into BOOT), and
//! [`ManifestBody::sig_scheme`] (which algorithm, see [`SigScheme`]). The
//! current CRC-only phase leaves the scheme at [`SigScheme::Unsigned`]
//! and `sig` all `0xFF`. When signing lands the signature covers the image
//! with its own bytes excluded (image start to `sig`, then past `sig` to
//! `image_len`), hashed with SHA-256; the CRCs stay as a cheap pre-check.
//! This layout is weak-frozen: stable for bench bring-up, ideally unchanged.

#[derive(bytemuck::AnyBitPattern, Clone, Copy)]
#[repr(C)]
/// Stored at [`crate::consts::MANIFEST_OFFSET`] inside the application image
pub struct AppManifest {
    pub head: ManifestHeader,
    pub body: ManifestBody,
}

/// The two CRC32s that cover the image, split around themselves: together
/// they cover the whole image except these eight bytes. `crc32_vec_table`
/// covers the vector table below the manifest (image start up to `head`);
/// `crc32_image` covers `body` through the end of the image (`image_len`).
#[derive(bytemuck::AnyBitPattern, Clone, Copy)]
#[repr(C)]
pub struct ManifestHeader {
    pub crc32_vec_table: u32,
    pub crc32_image: u32,
}

#[derive(bytemuck::AnyBitPattern, Clone, Copy)]
#[repr(C)]
/// ABI: new fields are APPEND ONLY, as this leaves us backwards-compatible.
pub struct ManifestBody {
    /// Selects the signing key among those baked into BOOT; 0 when unsigned.
    pub pubkey_id: u64,
    pub magic: u32,
    /// Word-aligned; bounds `crc32_image` and is range-checked in `Boot::verify`.
    pub image_len: u32,
    /// Anti-rollback counter (reserved for v2).
    pub version: u16,
    /// BOOT refuses an image below [`FMT_VER`].
    pub fmt_version: u16,
    /// [`SigScheme`] as a raw byte; read it typed with [`ManifestBody::scheme`].
    pub sig_scheme: u8,
    /// Must be 0: keeps the signed region free of implicit padding.
    pub reserved: [u8; 3],
    /// Detached signature; covers the image with its own bytes excluded.
    pub sig: [u8; 64],
}

impl ManifestBody {
    /// Typed view of [`sig_scheme`](Self::sig_scheme); `None` for an unknown
    /// (future) scheme, which an old BOOT must treat as unverifiable.
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
    /// Decode a stored [`ManifestBody::sig_scheme`] byte; `None` if it names
    /// no known scheme.
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Unsigned),
            1 => Some(Self::EcdsaP256Sha256),
            _ => None,
        }
    }
}

/// Manifest magic ([`ManifestBody::magic`]); the first check in verification.
pub const MAGIC: u32 = 0xa55af00b;
/// Current manifest layout version ([`ManifestBody::fmt_version`]).
pub const FMT_VER: u16 = 1;

// Weak-frozen layout: pin the size and the signed-region split point so a
// field reshuffle cannot silently move them. `sig` at body offset 24 keeps
// the struct 96 bytes with no implicit padding.
const _: () = assert!(size_of::<AppManifest>() == 96);
const _: () = assert!(core::mem::offset_of!(ManifestBody, sig) == 24);

impl AppManifest {
    /// An unsigned, unstamped manifest to reserve the slot with
    /// [`install_manifest!`](crate::install_manifest); the post-link tool
    /// fills the length, CRCs, and (when signing) the signature.
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
