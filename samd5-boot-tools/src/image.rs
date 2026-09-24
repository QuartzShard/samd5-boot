//! Making a linked application image acceptable to the bootloader.

use std::path::Path;

use anyhow::{Context, Result, bail};
use samd5_boot::manifest::{self, Stamped};

/// Stamp the manifest of a linked image, in place in `output`.
///
/// The application reserves the slot with `install_manifest!`; this fills in
/// the length and the two CRCs the bootloader verifies against, using the
/// same code and offsets it reads them back with. An image that has not been
/// through this will not verify.
///
/// The image is padded to a whole number of words first, because `image_len`
/// bounds a CRC the DSU walks in words.
pub fn stamp(input: &Path, output: &Path, version: u16) -> Result<Stamped> {
    let mut image = std::fs::read(input).with_context(|| format!("reading {}", input.display()))?;
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
    Ok(stamped)
}
