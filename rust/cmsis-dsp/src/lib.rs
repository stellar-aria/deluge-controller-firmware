//! Safe Rust wrapper around ARM CMSIS-DSP v1.17.0.
//!
//! Provides ergonomic, type-safe wrappers over the raw FFI bindings exposed by
//! `cmsis-dsp-sys`.  The API mirrors the CMSIS-DSP C API as closely as
//! practical while hiding raw pointers and status codes behind `Result`.
//!
//! # Feature flags
//! * `neon` *(default)* — enables `ARM_MATH_NEON` in the C library; the NEON
//!   API variants (3-buffer out-of-place) are selected automatically via
//!   `cfg(target_arch = "arm")` in the wrapper types.
//! * `loop-unroll` *(default)* — enables `ARM_MATH_LOOPUNROLL`.
//!
//! # `no_std`
//! This crate is `no_std` compatible when compiled for a bare-metal target.
//! Heap allocation is **not** required; all instance structs are stack- or
//! static-allocated by the caller.
#![no_std]

pub mod error;
pub mod fft;
pub mod math;
pub mod stats;

pub use error::Error;
