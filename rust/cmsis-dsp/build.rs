//! build.rs for cmsis-dsp
//!
//! Nothing to do at the moment; the crate uses cfg(target_arch = "arm")
//! to select the NEON vs scalar API, which is a built-in Rust cfg.

fn main() {}
