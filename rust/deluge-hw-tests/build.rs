use std::env;
use std::path::PathBuf;

fn main() {
    let target = env::var("TARGET").unwrap_or_default();

    // Find test.ld from the rza1 crate.
    let rza1_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap())
        .parent()
        .unwrap()
        .join("rza1");
    println!("cargo:rustc-link-search={}", rza1_dir.display());

    if target == "armv7a-none-eabihf" {
        println!("cargo:rustc-link-arg=-Ttest.ld");
    }

    println!("cargo:rerun-if-changed=build.rs");
}
