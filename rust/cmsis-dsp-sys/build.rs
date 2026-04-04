//! build.rs for cmsis-dsp-sys
//!
//! Steps
//! -----
//! 1. Download CMSIS-DSP v1.17.0 tarball from GitHub and verify its SHA-256.
//! 2. Extract the needed source tree into `OUT_DIR` (cached across builds).
//! 3. Compile the C sources with `cc`, enabling NEON when the target is ARM.
//! 4. Run `bindgen` against `wrapper.h` to emit `bindings.rs` into `OUT_DIR`.

use std::env;
use std::fs;
use std::path::{Path, PathBuf};

use sha2::{Digest, Sha256};

// ── Version & integrity ────────────────────────────────────────────────────

const VERSION: &str = "1.17.0";
const TARBALL_URL: &str =
    "https://github.com/ARM-software/CMSIS-DSP/archive/refs/tags/v1.17.0.tar.gz";
const TARBALL_SHA256: &str = "68f1f5706aa5785fc3c20d18bfe1eac7d363f460337d24bafe5999bee417434b";

// ── Source modules to compile (umbrella files, one per module) ─────────────
//
// Each entry expands to `Source/<MODULE>/<MODULE>.c` inside the CMSIS-DSP tree.
const MODULES: &[&str] = &[
    "BasicMathFunctions",
    "BayesFunctions",
    "CommonTables",
    "ComplexMathFunctions",
    "ControllerFunctions",
    "DistanceFunctions",
    "FastMathFunctions",
    "FilteringFunctions",
    "InterpolationFunctions",
    "QuaternionMathFunctions",
    "SVMFunctions",
    "StatisticsFunctions",
    "SupportFunctions",
    "TransformFunctions",
    "WindowFunctions",
];

// MatrixFunctions is compiled separately WITHOUT ARM_MATH_NEON because
// v1.17.0 contains a type-mismatch bug in its NEON path
// (_arm_mat_vec_mult_neon.c).  The fix landed on main after the release.
const MODULES_SKIP_NEON: &[&str] = &["MatrixFunctions"];

// ── Ne10 sources compiled only for ARM+NEON builds ────────────────────────
const NE10_SOURCES: &[&str] = &[
    // NEON twiddle/factor tables — required by arm_cfft_init_f32 when
    // ARM_MATH_NEON is defined; not included by the CommonTables.c umbrella.
    "Source/CommonTables/arm_neon_tables.c",
    "Ne10/CMSIS_NE10_fft_generic_float32.neonintrisic.c",
    "Ne10/CMSIS_NE10_fft_generic_int32.c",
    "Ne10/CMSIS_NE10_fft_generic_int32.neonintrisic.c",
    "Ne10/CMSIS_NE10_fft_init.c",
    "Ne10/NE10_fft_float32.neonintrinsic.c",
    "Ne10/NE10_fft_int32.neonintrinsic.c",
    "Ne10/NE10_rfft_float32.neonintrinsic.c",
    "Ne10/NE10_fft_int16.neonintrinsic.c",
    "ComputeLibrary/Source/arm_cl_tables.c",
];

// ────────────────────────────────────────────────────────────────────────────

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let target = env::var("TARGET").unwrap();
    let is_arm = target.starts_with("arm") || target.starts_with("aarch64");
    let neon = is_arm && cfg!(feature = "neon");

    // ── 1. Download & extract ─────────────────────────────────────────────
    let src_root = out_dir.join(format!("CMSIS-DSP-{VERSION}"));
    if !src_root.exists() {
        let tarball_path = out_dir.join(format!("cmsis-dsp-{VERSION}.tar.gz"));
        download_tarball(&tarball_path);
        verify_sha256(&tarball_path);
        extract_tarball(&tarball_path, &out_dir);
        // Sanity check.
        assert!(
            src_root.exists(),
            "Extraction failed: {} not found",
            src_root.display()
        );
    }

    // ── 2. Compile C library ──────────────────────────────────────────────
    compile_cmsis_dsp(&src_root, &out_dir, neon, &target);

    // ── 3. Generate bindings ──────────────────────────────────────────────
    generate_bindings(&src_root, &out_dir, neon);

    // ── 4. Linker instructions ────────────────────────────────────────────
    println!("cargo:rustc-link-search=native={}", out_dir.display());
    println!("cargo:rustc-link-lib=static=cmsis_dsp");

    // No custom cfg needed: callers detect NEON via cfg(target_arch = "arm").

    // Rebuild if wrapper.h changes.
    println!("cargo:rerun-if-changed=wrapper.h");
}

// ── Download ────────────────────────────────────────────────────────────────

fn download_tarball(dest: &Path) {
    use std::process::Command;
    eprintln!("Downloading CMSIS-DSP {VERSION}...");
    let status = Command::new("curl")
        .args([
            "--fail",
            "--silent",
            "--show-error",
            "--location",
            TARBALL_URL,
            "--output",
            dest.to_str().unwrap(),
        ])
        .status()
        .unwrap_or_else(|_| {
            // Fall back to wget if curl is not available.
            Command::new("wget")
                .args(["-q", "-O", dest.to_str().unwrap(), TARBALL_URL])
                .status()
                .expect("Neither curl nor wget found. Install one to download CMSIS-DSP.")
        });
    assert!(status.success(), "Download of CMSIS-DSP tarball failed");
}

// ── SHA-256 verification ─────────────────────────────────────────────────────

fn verify_sha256(path: &Path) {
    let bytes = fs::read(path).unwrap_or_else(|e| panic!("Cannot read {}: {e}", path.display()));
    let digest = Sha256::digest(&bytes);
    let hex_digest = hex::encode(digest);
    assert!(
        hex_digest == TARBALL_SHA256,
        "SHA-256 mismatch for CMSIS-DSP tarball!\n  expected: {TARBALL_SHA256}\n  got:      {hex_digest}"
    );
}

// ── Tarball extraction ───────────────────────────────────────────────────────

fn extract_tarball(tarball: &Path, dest: &Path) {
    use flate2::read::GzDecoder;
    use tar::Archive;

    eprintln!("Extracting CMSIS-DSP to {}...", dest.display());
    let file = fs::File::open(tarball).expect("Cannot open tarball");
    let gz = GzDecoder::new(file);
    let mut archive = Archive::new(gz);
    archive
        .unpack(dest)
        .expect("Failed to extract CMSIS-DSP tarball");
}

// ── C compilation ────────────────────────────────────────────────────────────

fn compile_cmsis_dsp(src_root: &Path, out_dir: &Path, neon: bool, target: &str) {
    let include_dir = src_root.join("Include");
    let priv_include_dir = src_root.join("PrivateInclude");
    let compute_inc = src_root.join("ComputeLibrary").join("Include");
    let ne10_dir = src_root.join("Ne10");

    let mut build = cc::Build::new();

    build
        .warnings(false)
        .include(&include_dir)
        .include(&priv_include_dir)
        // -D__GNUC_PYTHON__ makes CMSIS-DSP self-contained; no CMSIS-5 Core needed.
        .define("__GNUC_PYTHON__", None)
        // Disable f16: half-float requires specific compiler support.
        .define("DISABLEFLOAT16", None)
        .opt_level(3);

    // ffast-math is required by CMSIS-DSP for correct vectorisation.
    // This is safe here because it applies only to the C library, not Rust code.
    build.flag_if_supported("-ffast-math");

    if cfg!(feature = "loop-unroll") {
        build.define("ARM_MATH_LOOPUNROLL", None);
    }

    if neon {
        build
            .define("ARM_MATH_NEON", None)
            .include(&compute_inc)
            .include(&ne10_dir);

        // ARM-specific codegen flags (only meaningful when the C compiler
        // is already targeting ARM, which cc handles via CC / AR env vars).
        if target.starts_with("arm") {
            build
                .flag_if_supported("-mfpu=neon-fp16")
                .flag_if_supported("-mfloat-abi=hard");
        }
    }

    // Compile umbrella files (each includes all _f32.c, _q31.c … siblings).
    for module in MODULES {
        let src = src_root
            .join("Source")
            .join(module)
            .join(format!("{module}.c"));
        if src.exists() {
            build.file(&src);
        }
    }

    if neon {
        for rel in NE10_SOURCES {
            let src = src_root.join(rel);
            if src.exists() {
                build.file(&src);
            }
        }
    }

    build.out_dir(out_dir).compile("cmsis_dsp");

    // Compile NEON-bugged modules without ARM_MATH_NEON (v1.17.0 workaround).
    if neon && !MODULES_SKIP_NEON.is_empty() {
        let mut compat = cc::Build::new();
        compat
            .warnings(false)
            .include(&include_dir)
            .include(&priv_include_dir)
            .define("__GNUC_PYTHON__", None)
            .define("DISABLEFLOAT16", None)
            .opt_level(3);
        compat.flag_if_supported("-ffast-math");
        if cfg!(feature = "loop-unroll") {
            compat.define("ARM_MATH_LOOPUNROLL", None);
        }
        // No ARM_MATH_NEON here — avoiding the buggy matrix NEON code.
        for module in MODULES_SKIP_NEON {
            let src = src_root
                .join("Source")
                .join(module)
                .join(format!("{module}.c"));
            if src.exists() {
                compat.file(&src);
            }
        }
        // The compat objects are linked into the same archive by using a
        // unique output name then merging with ar.  Simplest approach: compile
        // as a separate static lib and emit an additional link directive.
        compat.out_dir(out_dir).compile("cmsis_dsp_compat");
        // Note: cc::Build::compile() already emits cargo:rustc-link-lib for us.
    }
}

// ── Bindgen ──────────────────────────────────────────────────────────────────

fn generate_bindings(src_root: &Path, out_dir: &Path, neon: bool) {
    let include_dir = src_root.join("Include");
    let priv_include_dir = src_root.join("PrivateInclude");
    let compute_inc = src_root.join("ComputeLibrary").join("Include");
    let ne10_dir = src_root.join("Ne10");

    let target = env::var("TARGET").unwrap();
    let is_arm = target.starts_with("arm") || target.starts_with("aarch64");

    let wrapper = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("wrapper.h");

    let mut builder = bindgen::Builder::default()
        .header(wrapper.to_str().unwrap())
        .clang_arg(format!("-I{}", include_dir.display()))
        .clang_arg(format!("-I{}", priv_include_dir.display()))
        .clang_arg("-D__GNUC_PYTHON__")
        .clang_arg("-DDISABLEFLOAT16")
        // Keep only CMSIS-DSP symbols; skip OS / compiler internals.
        .allowlist_function("arm_.*")
        .allowlist_type("arm_.*|q7_t|q15_t|q31_t|q63_t|float16_t|float32_t|float64_t")
        .allowlist_var("arm_.*")
        .use_core()
        .ctypes_prefix("::core::ffi")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .layout_tests(false);

    // For ARM targets, tell clang to use the Linux ARM userspace triple.
    // This makes standard headers (string.h, stdint.h …) available while
    // keeping identical struct layouts to the bare-metal target.
    if is_arm {
        builder = builder.clang_arg("--target=arm-linux-gnueabihf");
    }

    if neon {
        builder = builder
            .clang_arg("-DARM_MATH_NEON")
            .clang_arg(format!("-I{}", compute_inc.display()))
            .clang_arg(format!("-I{}", ne10_dir.display()));
    }

    if cfg!(feature = "loop-unroll") {
        builder = builder.clang_arg("-DARM_MATH_LOOPUNROLL");
    }

    let bindings = builder
        .generate()
        .expect("bindgen failed to generate CMSIS-DSP bindings");

    let out_path = out_dir.join("bindings.rs");
    bindings
        .write_to_file(&out_path)
        .expect("Failed to write bindings.rs");
}
