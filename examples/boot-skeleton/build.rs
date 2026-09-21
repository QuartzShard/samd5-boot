//! A downstream binary owns its memory.x and puts it where the linker
//! finds it. `samd5-boot`'s own build script emits `samd5_boot_boot.x`
//! (which memory.x INCLUDEs) onto the link search path.

use std::{env, fs, path::PathBuf};

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::copy("memory.x", out.join("memory.x")).unwrap();
    println!("cargo::rustc-link-search={}", out.display());
    println!("cargo::rerun-if-changed=memory.x");
}
