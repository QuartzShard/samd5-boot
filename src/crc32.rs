//! CRC-32/ISO-HDLC (zlib crc32), the convention the SAMD5x DSU computes
//!
//! [`crc32`] is the whole module. Host tooling stamps the manifest with it
//! and BOOT verifies the same bytes with the DSU, so the two must agree byte
//! for byte; the test vectors below pin that.
//!
//! Parameters: width 32, poly 0x04C11DB7, init 0xFFFFFFFF, refin = true,
//! refout = true, xorout = 0xFFFFFFFF. Because input and output are
//! reflected, the implementation uses the bit-reversed poly 0xEDB88320 and
//! processes each byte LSB-first, which is the standard byte-at-a-time
//! reflected form. The DSU walks memory in ascending-address order; feeding
//! the same bytes in the same order reproduces its result exactly. The hal
//! complements the DSU's `DATA` register, so `Dsu::crc32` returns this same
//! value.
//!
//! [`crate::persist::BootStore`] seals itself with the same convention but a
//! table-free loop, so a BOOT that never calls [`crc32`] does not link the
//! 1 KiB table. That holds only while BOOT is built for size: at opt-level 3
//! LLVM's CRC loop-idiom pass rebuilds the table from the loop.

const POLY_REFLECTED: u32 = 0xEDB8_8320;

/// Byte-wise lookup table for the reflected polynomial, built at compile time
const TABLE: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut n = 0usize;
    while n < 256 {
        let mut c = n as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 {
                POLY_REFLECTED ^ (c >> 1)
            } else {
                c >> 1
            };
            k += 1;
        }
        table[n] = c;
        n += 1;
    }
    table
};

/// Compute the CRC-32 of `data`
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc = TABLE[((crc ^ b as u32) & 0xFF) as usize] ^ (crc >> 8);
    }
    crc ^ 0xFFFF_FFFF
}

#[cfg(test)]
mod tests {
    use super::crc32;

    // Independent, well-known zlib crc32 reference values. The "123456789"
    // check value 0xCBF43926 is the canonical CRC-32/ISO-HDLC vector and is
    // what pins this implementation to the DSU's convention forever.
    #[test]
    fn known_vectors() {
        assert_eq!(crc32(b""), 0x0000_0000);
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b"a"), 0xE8B7_BE43);
        assert_eq!(crc32(b"abc"), 0x3524_41C2);
        assert_eq!(
            crc32(b"The quick brown fox jumps over the lazy dog"),
            0x414F_A339
        );
    }

    // A fixed byte -> fixed CRC vector over a longer, arbitrary buffer, so the
    // convention stays pinned for non-ASCII / word-aligned inputs too.
    #[test]
    fn fixed_binary_vector() {
        let buf: [u8; 16] = [
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD,
            0xFE, 0xFF,
        ];
        assert_eq!(crc32(&buf), 0x92DE_F389);
    }

    // An all-0xFF word: what an erased/unstamped region looks like.
    #[test]
    fn erased_word() {
        assert_eq!(crc32(&[0xFF, 0xFF, 0xFF, 0xFF]), 0xFFFF_FFFF);
    }
}
