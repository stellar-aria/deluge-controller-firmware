#![cfg_attr(target_os = "none", no_std)]

// Startup lives in rza1::startup. When rza1 is linked into any binary,
// startup.rs is included automatically because _start and the vector table
// are referenced by the linker script.

pub mod uart;
