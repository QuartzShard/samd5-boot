#[derive(bytemuck::AnyBitPattern, Clone, Copy)]
#[repr(C)]
/// Stored at [`crate::consts::MANIFEST_OFFSET`] inside the application image
pub struct AppManifest {
    pub head: ManifestHeader,
    pub body: ManifestBody,
}

#[derive(bytemuck::AnyBitPattern, Clone, Copy)]
#[repr(C)]
pub struct ManifestHeader {
    pub crc32_vec_table: u32, // IMAGE_START --> manifest.head
    pub crc32_image: u32,     // manifest.body --> IMAGE_END
}

#[derive(bytemuck::AnyBitPattern, Clone, Copy)]
#[repr(C)]
/// ABI: new fields are APPEND ONLY, as this leaves us backwards-compatible
pub struct ManifestBody {
    pub pubkey_id: u64,
    pub magic: u32,
    pub image_len: u32,
    pub version: u16,
    pub fmt_version: u16,
    pub sig: [u8; 64],
}

pub const MAGIC: u32 = 0xa55af00b;
pub const FMT_VER: u16 = 1;

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
