#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::panic::PanicInfo;
use rtt_target::{rprintln, rtt_init_print};

use core::mem::MaybeUninit;

use embassy_executor::{Executor, Spawner};
use embassy_time::Timer;

use rza1::{gic, ostm, cache, mmu, sdram, ssi, stb};
use deluge_bsp::uart as bsp_uart;

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
        Timer::after_millis(500).await;
    }
}

/// Continuous audio output task.
///
/// Tracks the DMA read pointer and refills the TX buffer ahead of it with a
/// 1 kHz test tone (square wave at ±25% full scale).  At 44.1 kHz with a
/// 1 024-frame circular buffer (~23 ms), the task refills 256 frames every
/// ~3 ms, staying comfortably ahead of the DMA.
///
/// Replace the tone generator here with real DSP once the hardware is verified.
#[embassy_executor::task]
async fn audio_task() {
    // Period of the test tone in stereo half-frames.
    // 1 kHz → 44 100 / 1000 = 44 frames per cycle; half-period = 22 frames.
    const HALF_PERIOD: usize = 22;
    const AMPLITUDE:   i32   = i32::MAX / 4; // ±25 % full scale

    let buf_start = ssi::tx_buf_start();
    let buf_end   = ssi::tx_buf_end();

    rprintln!("Audio task started. TX buffer {:p}..{:p}", buf_start, buf_end);

    // Let the DMA engine warm up before we start tracking its position.
    Timer::after_millis(5).await;

    let mut write_ptr: *mut i32 = buf_start;
    let mut phase:     usize    = 0;

    loop {
        // Fill 256 stereo frames (512 samples) per iteration ≈ 5.8 ms of audio.
        // The buffer is 1 024 frames deep, so we stay well ahead of the DMA.
        for _ in 0..256usize {
            let sample = if phase < HALF_PERIOD { AMPLITUDE } else { -AMPLITUDE };
            phase = (phase + 1) % (HALF_PERIOD * 2);

            unsafe {
                // Left channel
                write_ptr.write_volatile(sample);
                write_ptr = write_ptr.add(1);
                if write_ptr >= buf_end {
                    write_ptr = buf_start;
                }
                // Right channel (same as left for a mono test tone)
                write_ptr.write_volatile(sample);
                write_ptr = write_ptr.add(1);
                if write_ptr >= buf_end {
                    write_ptr = buf_start;
                }
            }
        }

        // Sleep until just before the DMA catches up (~3 ms per 256-frame chunk).
        Timer::after_millis(3).await;
    }
}

/// MIDI DIN loopback: echo every received byte straight back.
///
/// Useful for hardware verification: jumper P6_15 (TX) → P6_14 (RX).
#[embassy_executor::task]
async fn midi_echo_task() {
    rprintln!("MIDI echo task started (SCIF0 @ 31250 baud)");
    loop {
        let byte = bsp_uart::read_byte(bsp_uart::MIDI_CH).await;
        bsp_uart::write_bytes(bsp_uart::MIDI_CH, &[byte]).await;
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

static mut EXECUTOR: MaybeUninit<Executor> = MaybeUninit::uninit();

/// Entry point called from startup assembly after BSS clear and stack setup.
#[no_mangle]
pub extern "C" fn main() -> ! {
    rtt_init_print!();
    rprintln!("Deluge Rust firmware starting");
    rprintln!("CPU: Renesas RZ/A1L Cortex-A9 @ 400 MHz");

    // ---- 1. Enable peripheral module clocks (CPG STBCRn) -----------------
    unsafe { stb::init() };
    rprintln!("STB: all module clocks enabled");

    // ---- 2. Build L1 Translation Table and enable MMU --------------------
    unsafe { mmu::init_and_enable() };
    rprintln!("MMU enabled (flat VA=PA, 4 GB mapping)");

    // ---- 3. Enable L1 I+D caches, branch prediction, D-prefetch ----------
    unsafe { cache::l1_enable() };
    rprintln!("L1 cache enabled");

    // ---- 4. Initialise and enable L2 cache (PL310) -----------------------
    unsafe { cache::l2_init() };
    rprintln!("L2 cache enabled (D-ways locked, I-ways open)");

    // ---- 5. Initialise SDRAM (pin-mux + BSC) -----------------------------
    unsafe { sdram::init() };
    rprintln!("SDRAM initialised (64 MB @ 0x0C000000)");

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

    // ---- UART (SCIF) init ------------------------------------------------
    unsafe {
        bsp_uart::init_midi(31250); // SCIF0 — MIDI DIN, P6_15/P6_14
        bsp_uart::init_pic(31250);  // SCIF1 — PIC32,    P3_15/P1_9
    }
    rprintln!("UART SCIF0/1 initialized at 31250 baud");

    // ---- Audio: SSI0 pin-mux, DMA init, codec power-on -------------------
    unsafe { deluge_bsp::audio::init() };
    rprintln!("Audio: SSI0 + DMA running, codec enabled");

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
        spawner.spawn(audio_task().unwrap());
        spawner.spawn(midi_echo_task().unwrap());
    });
}
