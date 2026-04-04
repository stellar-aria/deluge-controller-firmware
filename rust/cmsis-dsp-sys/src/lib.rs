//! Raw FFI bindings to ARM CMSIS-DSP v1.17.0.
//!
//! This crate is `no_std`-compatible.  All symbols are re-exported from the
//! `bindgen`-generated `bindings.rs` file produced at build time.
//!
//! # Safety
//! All functions here are `unsafe` C FFI calls.  Prefer the higher-level
//! `cmsis-dsp` crate which provides safe Rust wrappers.
#![no_std]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(dead_code)]
#![allow(clippy::all)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
