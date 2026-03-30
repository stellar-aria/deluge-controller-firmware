use std::env;
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let target = env::var("TARGET").unwrap_or_default();

    // Make test.ld visible to the linker for test binaries.
    // cargo:rustc-link-search is also propagated to packages that depend on
    // rza1, so deluge-bsp test binaries can find test.ld here too.
    println!("cargo:rustc-link-search={}", manifest_dir.display());

    // cargo:rustc-link-arg applies to this package's own binary outputs
    // (including integration test binaries in tests/) but is NOT propagated
    // to other packages (firmware), so it is safe to emit unconditionally for
    // the embedded target.  The firmware binary picks up -Tlinker.ld from
    // firmware/build.rs instead.
    if target == "armv7a-none-eabihf" {
        println!("cargo:rustc-link-arg=-Ttest.ld");
    }

    println!("cargo:rerun-if-changed=src/startup.rs");
    println!("cargo:rerun-if-changed=test.ld");
}
