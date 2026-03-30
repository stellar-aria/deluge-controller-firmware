use std::env;
use std::path::PathBuf;

fn main() {
    let target = env::var("TARGET").unwrap_or_default();

    // test.ld lives in the rza1 crate (workspace sibling).
    // rza1's build.rs also emits this search path, but we add it here
    // explicitly so it's available when linking deluge-bsp test binaries
    // (Cargo does not guarantee propagation of rustc-link-search across
    // package boundaries in all scenarios).
    let rza1_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap())
        .parent()
        .unwrap()
        .join("rza1");
    println!("cargo:rustc-link-search={}", rza1_dir.display());

    // Same rationale as rza1/build.rs: applies to this package's test binaries
    // only (not firmware, which is a different package).
    if target == "armv7a-none-eabihf" {
        println!("cargo:rustc-link-arg=-Ttest.ld");
    }

    println!("cargo:rerun-if-changed=build.rs");
}
