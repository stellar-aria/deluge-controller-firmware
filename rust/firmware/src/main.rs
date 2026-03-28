#![no_std]
#![no_main]

mod startup;

use core::panic::PanicInfo;
use rtt_target::{rprintln, rtt_init_print};

use rza1::gic;
use rza1::ostm;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("PANIC: {}", info);
    loop {
        core::hint::spin_loop();
    }
}

/// Firmware entry point, called from startup assembly after BSS is zeroed.
#[no_mangle]
pub extern "C" fn rust_main() -> ! {
    rtt_init_print!();
    rprintln!("Deluge Rust firmware starting");
    rprintln!("CPU: Renesas RZ/A1L Cortex-A9 @ 400 MHz");

    // ---- Interrupt infrastructure ----------------------------------------
    // Initialize GIC distributor + CPU interface. All sources start masked.
    unsafe { gic::init() };
    rprintln!("GIC initialized");

    // ---- OSTM free-running timer -----------------------------------------
    // Ungate OSTM0+OSTM1 clocks in CPG, then start OSTM0 as a free-running
    // 32-bit counter at P0 = 33.33 MHz (≈30 ns per tick).
    unsafe {
        ostm::enable_clock();
        ostm::start_free_running(0);
    }
    rprintln!("OSTM0 running at ~33.33 MHz");

    // ---- GPIO output: SYNC LED (P6_7) ------------------------------------
    // SYNCED_LED = {6, 7} per definitions_cxx.hpp
    const SYNC_PORT: u8 = 6;
    const SYNC_PIN: u8 = 7;
    unsafe { rza1::gpio::set_as_output(SYNC_PORT, SYNC_PIN) };
    rprintln!("SYNC LED (P6_7) configured as output");

    // ---- Enable global IRQ -----------------------------------------------
    // No interrupt sources are enabled yet, so this is safe. Future drivers
    // call gic::register() + gic::enable() before their peripheral starts.
    unsafe { core::arch::asm!("cpsie i", options(nomem, nostack)) };
    rprintln!("IRQ enabled — blink loop starting");

    // ---- Blink loop using OSTM timing ------------------------------------
    // 500 ms on / 500 ms off at P0 = 33.33 MHz → 16_666_666 ticks per half-period
    const HALF_PERIOD_TICKS: u32 = ostm::OSTM_HZ / 2;

    let mut led_on = false;
    loop {
        led_on = !led_on;
        unsafe { rza1::gpio::write(SYNC_PORT, SYNC_PIN, led_on) };
        rprintln!("SYNC LED {}", if led_on { "ON" } else { "OFF" });
        unsafe { ostm::delay_ticks(HALF_PERIOD_TICKS) };
    }
}


