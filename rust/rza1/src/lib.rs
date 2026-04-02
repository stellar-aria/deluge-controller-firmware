//! `rza1` — Embassy HAL for the Renesas RZ/A1L (Cortex-A9, R7S721001).
//!
//! Peripheral base addresses are taken from the RZ/A1L Hardware Manual.

#![cfg_attr(target_os = "none", no_std)]
#![feature(allocator_api)]

// startup, mmu and cache use ARM coprocessor assembly and cannot compile on the host.
pub mod bsc;
#[cfg(target_os = "none")]
pub mod cache;
pub mod dmac;
pub mod gic;
#[cfg(target_os = "none")]
pub mod mmu;
pub mod mtu2;
pub mod ostm;
pub mod rspi;
pub mod rusb1;
pub mod sdhi;
pub mod sdram;
pub mod ssi;
#[cfg(target_os = "none")]
pub mod startup;
pub mod stb;
pub mod time_driver;
pub mod uart;

pub mod allocator;
pub mod gpio;
