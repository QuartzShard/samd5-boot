//! Emits `density` / `boot_size` cfgs from the selected features.
//! Target selection lists each part exactly once (here). The role linker
//! fragments are written to OUT_DIR and found through `rustc-link-search`;
//! see the crate docs for how a downstream memory.x pulls one in.
//!
//! Products using SmartEEPROM set `SAMD5_BOOT_SEE_SBLK` (the SBLK fuse
//! value, default 0) so the app region's top drops by the SEE reserve.
//! Part-to-density mapping per DS60001507 Tables 1-1 / 1-2.

use std::{env, fs, path::PathBuf};

/// (density digit, RAM size, parts): the digit is log2 of the flash size
/// per Microchip part numbering, so flash is derived as `1 << digit`.
const DENSITIES: &[(u32, usize, &[&str])] = &[
    (
        18,
        128 * 1024,
        &[
            "samd51g18a",
            "samd51j18a",
            "same51g18a",
            "same51j18a",
            "same53j18a",
        ],
    ),
    (
        19,
        192 * 1024,
        &[
            "samd51g19a",
            "samd51j19a",
            "samd51n19a",
            "samd51p19a",
            "same51g19a",
            "same51j19a",
            "same51n19a",
            "same53j19a",
            "same53n19a",
            "same54n19a",
            "same54p19a",
        ],
    ),
    (
        20,
        256 * 1024,
        &[
            "samd51j20a",
            "samd51n20a",
            "samd51p20a",
            "same51j20a",
            "same51n20a",
            "same53j20a",
            "same53n20a",
            "same54n20a",
            "same54p20a",
        ],
    ),
];

fn main() {
    println!("cargo::rerun-if-changed=build.rs");

    println!("cargo::rustc-check-cfg=cfg(density, values(\"18\", \"19\", \"20\"))");
    println!("cargo::rustc-check-cfg=cfg(density_known)");
    let selected_densities: Vec<(u32, usize)> = DENSITIES
        .iter()
        .filter(|(_, _, parts)| {
            parts
                .iter()
                .any(|p| env::var(format!("CARGO_FEATURE_{}", p.to_uppercase())).is_ok())
        })
        .map(|&(digit, ram, _)| (digit, ram))
        .collect();
    for (digit, _) in &selected_densities {
        println!("cargo::rustc-cfg=density=\"{digit}\"");
    }
    if !selected_densities.is_empty() {
        println!("cargo::rustc-cfg=density_known");
    }

    println!("cargo::rustc-check-cfg=cfg(boot_size, values(\"16k\", \"32k\", \"64k\", \"96k\"))");
    let selected: Vec<&str> = ["16k", "32k", "64k", "96k"]
        .into_iter()
        .filter(|s| env::var(format!("CARGO_FEATURE_BOOTPROT_{}", s.to_uppercase())).is_ok())
        .collect();
    let boot_size = match selected.as_slice() {
        [] => "32k",
        [one] => one,
        many => panic!(
            "samd5-boot: at most one bootprot-* feature, got: bootprot-{}",
            many.join(", bootprot-")
        ),
    };
    println!("cargo::rustc-cfg=boot_size=\"{boot_size}\"");

    // Linker scripts need an unambiguous part; otherwise skip generation
    // and let the compile_error in consts.rs be the diagnostic.
    let &[(digit, ram_size)] = selected_densities.as_slice() else {
        return;
    };
    let bank_size = (1usize << digit) / 2;
    let boot_len = 1024
        * boot_size
            .trim_end_matches('k')
            .parse::<usize>()
            .expect("bootprot feature names are <n>k");

    println!("cargo::rerun-if-env-changed=SAMD5_BOOT_SEE_SBLK");
    let sblk: usize = match env::var("SAMD5_BOOT_SEE_SBLK") {
        Ok(v) => match v.parse() {
            Ok(s) if s <= 10 => s,
            _ => panic!("samd5-boot: SAMD5_BOOT_SEE_SBLK must be 0..=10, got {v:?}"),
        },
        Err(_) => 0,
    };
    let see_reserve = 2 * sblk * 8192;
    let app_len = bank_size
        .checked_sub(boot_len + see_reserve)
        .filter(|l| *l > 0)
        .unwrap_or_else(|| {
            panic!(
                "samd5-boot: BOOT ({boot_len} B) + SEE reserve ({see_reserve} B) \
                 leave no app region in a {bank_size} B bank"
            )
        });

    // Carve the top page (consts::PAGE_SIZE) of the BOOT region into its own
    // region for the boot-info block: cortex-m-rt's FLASH then stops below it,
    // so BOOT code can never grow over the block, and the block sits in a
    // region whose cursor is independent of FLASH (a single FLASH region with
    // a top-pinned section trips lld's monotonic region cursor). Its origin is
    // consts::BOOT_INFO_ADDR, where the application reads it.
    let info_reserve = 512;
    let boot_code_len = boot_len - info_reserve;
    let boot_info_addr = boot_code_len;

    let boot_x = format!(
        "/* samd5-boot BOOT image: active-slot base (each bank's head),\n\
         \x20  BOOTPROT-sized. The top page (BOOTINFO) holds the boot-info\n\
         \x20  block the application reads; FLASH is the BOOT code below it.\n\
         \x20  The BOOT binary's memory.x selects this role with the single\n\
         \x20  line `INCLUDE samd5_boot_boot.x`. */\n\
         MEMORY\n\
         {{\n\
         \x20 FLASH    : ORIGIN = 0x00000000, LENGTH = {boot_code_len}\n\
         \x20 BOOTINFO : ORIGIN = {boot_info_addr:#x}, LENGTH = {info_reserve}\n\
         \x20 RAM      : ORIGIN = 0x20000000, LENGTH = {ram_size}\n\
         }}\n\
         \n\
         /* Frozen flash ABI (consts::BOOT_INFO_ADDR): the application reads\n\
         \x20  the boot-info block at a fixed absolute address, the top page\n\
         \x20  of the BOOT region. */\n\
         SECTIONS\n\
         {{\n\
         \x20 .samd5_boot_info ORIGIN(BOOTINFO) :\n\
         \x20 {{\n\
         \x20   KEEP(*(.samd5_boot_info));\n\
         \x20 }} > BOOTINFO\n\
         }} INSERT AFTER .vector_table;\n"
    );

    let app_x = format!(
        "/* samd5-boot application image: the active-slot app region.\n\
         \x20  The app binary's memory.x selects this role with the single\n\
         \x20  line `INCLUDE samd5_boot_app.x`. */\n\
         MEMORY\n\
         {{\n\
         \x20 FLASH : ORIGIN = {boot_len:#x}, LENGTH = {app_len}\n\
         \x20 RAM   : ORIGIN = 0x20000000, LENGTH = {ram_size}\n\
         }}\n\
         \n\
         /* Frozen flash ABI v1 (consts::MANIFEST_OFFSET): BOOT reads the\n\
         \x20  manifest at this fixed offset; code starts wherever the\n\
         \x20  manifest section actually ends. */\n\
         SECTIONS\n\
         {{\n\
         \x20 .samd5_boot_manifest ORIGIN(FLASH) + 0x400 :\n\
         \x20 {{\n\
         \x20   KEEP(*(.samd5_boot_manifest));\n\
         \x20 }} > FLASH\n\
         }} INSERT AFTER .vector_table;\n\
         \n\
         _stext = ADDR(.samd5_boot_manifest) + SIZEOF(.samd5_boot_manifest);\n"
    );

    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::write(out.join("samd5_boot_boot.x"), boot_x).unwrap();
    fs::write(out.join("samd5_boot_app.x"), app_x).unwrap();
    println!("cargo::rustc-link-search={}", out.display());
}
