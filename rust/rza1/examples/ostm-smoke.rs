//! Standalone OSTM smoke test — no embedded-test, no semihosting.
//!
//! This binary starts OSTM0 in free-running mode, reads six counter
//! snapshots via RTT, then executes an undefined instruction to halt
//! the core and stop probe-rs.
//!
//! Purpose: verify probe-rs ↔ RZ/A1L communication and OSTM operation
//! independently of the embedded-test framework.
//!
//! Run: `cargo test-fw -p rza1 --example ostm-smoke`
#![no_std]
#![no_main]

use rza1::{ostm, stb};
use rtt_target::{rprintln, rtt_init_print};

// Pull in the startup (vector table, reset handler, BSS zero, stack setup)
use rza1 as _;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    // Halt via undefined instruction so probe-rs stops cleanly.
    unsafe { core::arch::asm!("udf #1", options(nostack, nomem, noreturn)) }
}

#[no_mangle]
pub extern "C" fn main() -> ! {
    unsafe {
        rtt_init_print!();

        rprintln!("[ostm-smoke] starting");

        stb::init();
        ostm::enable_clock();

        // Read register state before start
        let base = 0xFCFEC000usize;
        rprintln!("[ostm-smoke] pre-start: CTL={:#04x} TE={} CNT={:#010x}",
            ((base + 0x20) as *const u8).read_volatile(),
            ((base + 0x10) as *const u8).read_volatile(),
            ((base + 0x04) as *const u32).read_volatile(),
        );

        ostm::start_free_running(0);

        let ctl = ((base + 0x20) as *const u8).read_volatile();
        let te  = ((base + 0x10) as *const u8).read_volatile();
        rprintln!("[ostm-smoke] post-start: CTL={:#04x} TE={}", ctl, te);

        // Print six counter snapshots ~1 ms apart
        for i in 0..6u32 {
            let t0 = ostm::count(0);
            // spin ~1 ms worth of ticks
            let target = ostm::OSTM_HZ / 1000;
            let mut t = ostm::count(0);
            while t.wrapping_sub(t0) < target {
                t = ostm::count(0);
            }
            rprintln!("[ostm-smoke] sample {}: CNT={:#010x}  delta={}", i, t, t.wrapping_sub(t0));
        }

        rprintln!("[ostm-smoke] done — halting via UDF");

        core::arch::asm!("udf #0", options(nostack, nomem, noreturn));
    }
}
