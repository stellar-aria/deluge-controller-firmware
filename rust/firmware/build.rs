use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());

    let linker_src = manifest_dir.join("linker.ld");
    fs::copy(&linker_src, out_dir.join("linker.ld")).unwrap();

    println!("cargo:rustc-link-search={}", out_dir.display());
    // Apply the firmware linker script. This lives in firmware/build.rs (not
    // the workspace .cargo/config.toml) so test binaries link with test.ld
    // from deluge-bsp instead.
    println!("cargo:rustc-link-arg=-Tlinker.ld");
    println!("cargo:rerun-if-changed=linker.ld");


}
