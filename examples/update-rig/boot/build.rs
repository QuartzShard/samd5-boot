//! Puts this binary's memory.x where the linker finds it; samd5-boot's own
//! build script emits the `samd5_boot_boot.x` it INCLUDEs onto the same path.

use std::{env, fs, path::PathBuf};

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::copy("memory.x", out.join("memory.x")).unwrap();
    println!("cargo::rustc-link-search={}", out.display());
    println!("cargo::rerun-if-changed=memory.x");
}
