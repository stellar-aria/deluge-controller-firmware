#![no_std]
#![no_main]

mod startup;

use core::panic::PanicInfo;
use rtt_target::{rprintln, rtt_init_print};

// Pull in rza1 to link its critical-section implementation
use rza1 as _;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("PANIC: {}", info);
    loop {
        core::hint::spin_loop();
    }
}

/// Spin for approximately `n` NOP cycles — rough busy-wait delay.
/// At 400 MHz (no cache, internal SRAM) each iteration is ~2–4 cycles,
/// so `n = 20_000_000` ≈ 100–200 ms.
#[inline(never)]
fn delay(n: u32) {
    for _ in 0..n {
        unsafe { core::arch::asm!("nop", options(nomem, nostack, preserves_flags)) };
    }
}

/// Firmware entry point, called from startup assembly after BSS is zeroed.
#[no_mangle]
pub extern "C" fn rust_main() -> ! {
    rtt_init_print!();
    rprintln!("Deluge Rust firmware starting");
    rprintln!("CPU: Renesas RZ/A1L Cortex-A9 @ 400 MHz");

    // SYNCED_LED = P6_7  (matches C firmware: definitions_cxx.hpp `constexpr Pin SYNCED_LED = {6, 7}`)
    const SYNC_PORT: u8 = 6;
    const SYNC_PIN: u8 = 7;

    // Configure P6_7 as a GPIO output (PMC=0, PM=0, PIPC=0)
    unsafe { rza1::gpio::set_as_output(SYNC_PORT, SYNC_PIN) };
    rprintln!("SYNC LED (P6_7) configured as output — starting blink");

    let mut led_on = false;
    loop {
        led_on = !led_on;
        unsafe { rza1::gpio::write(SYNC_PORT, SYNC_PIN, led_on) };
        rprintln!("SYNC LED {}", if led_on { "ON" } else { "OFF" });
        delay(20_000_000); // ~100–200 ms
    }
}

