#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod startup;

use core::panic::PanicInfo;
use rtt_target::{rprintln, rtt_init_print};

use core::mem::MaybeUninit;

use embassy_executor::{Executor, Spawner};
use embassy_time::Timer;

use rza1::gic;
use rza1::ostm;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("PANIC: {}", info);
    loop {
        core::hint::spin_loop();
    }
}

// ---------------------------------------------------------------------------
// Async tasks
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn blink_task() {
    const SYNC_PORT: u8 = 6;
    const SYNC_PIN: u8 = 7;
    let mut led_on = false;
    loop {
        led_on = !led_on;
        unsafe { rza1::gpio::write(SYNC_PORT, SYNC_PIN, led_on) };
        rprintln!("SYNC LED {}", if led_on { "ON" } else { "OFF" });
        Timer::after_millis(500).await;
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

static mut EXECUTOR: MaybeUninit<Executor> = MaybeUninit::uninit();

/// Entry point called from startup assembly after BSS clear and stack setup.
#[no_mangle]
pub extern "C" fn rust_main() -> ! {
    rtt_init_print!();
    rprintln!("Deluge Rust firmware starting");
    rprintln!("CPU: Renesas RZ/A1L Cortex-A9 @ 400 MHz");

    // ---- Interrupt infrastructure ----------------------------------------
    unsafe { gic::init() };
    rprintln!("GIC initialized");

    // ---- OSTM clocks on, then Embassy time driver ------------------------
    unsafe {
        ostm::enable_clock();
        rza1::time_driver::init();
    }
    rprintln!("Embassy time driver ready (OSTM0/1)");

    // ---- GPIO output: SYNC LED (P6_7) ------------------------------------
    unsafe { rza1::gpio::set_as_output(6, 7) };
    rprintln!("SYNC LED (P6_7) configured as output");

    // ---- Enable global IRQ -----------------------------------------------
    unsafe { cortex_ar::interrupt::enable() };
    rprintln!("IRQ enabled — starting Embassy executor");

    // ---- Embassy executor ------------------------------------------------
    // Safety: single-core, single-threaded; EXECUTOR is written exactly once
    // before `executor.run()` takes the exclusive reference forever.
    #[allow(static_mut_refs)]
    let executor = unsafe {
        EXECUTOR.write(Executor::new());
        EXECUTOR.assume_init_mut()
    };
    executor.run(|spawner: Spawner| {
        spawner.spawn(blink_task().unwrap());
    });
}
