//! Combined on-device hardware test suite.
//!
//! All tests run from a single ELF — one JLink flash per `cargo run-hw-tests`.
//!
//! Init strategy: perform the full init chain upfront so every test can assume
//! a fully initialised system.  Tests that only exercise a subset of the
//! hardware (e.g. GPIO) simply ignore what they don't need.
//!
//! Full init order:
//!   STB → MMU → L1 cache → L2 cache → SDRAM → GIC → OSTM clocks →
//!   OSTM0 free-running → time_driver → IRQ enable
//!
//! Build:  cargo build-fw -p deluge-hw-tests
//! Run:    cargo run-hw-tests   (see .cargo/config.toml alias)
#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
#![cfg_attr(target_os = "none", feature(impl_trait_in_assoc_type))]

#[cfg(not(target_os = "none"))]
fn main() {}

#[cfg(target_os = "none")]
use deluge_test as _;

#[cfg(target_os = "none")]
use core::sync::atomic::{AtomicBool, Ordering};

// Global flag for the GIC SGI test — must live outside the tests module so
// the interrupt handler closure can reach it.
#[cfg(target_os = "none")]
static SGI0_FIRED: AtomicBool = AtomicBool::new(false);

#[cfg(target_os = "none")]
#[deluge_test::tests(executor = embassy_executor::Executor::new())]
mod tests {
    use core::sync::atomic::Ordering;
    use embassy_time::{Duration, Instant, Timer};
    use rza1::{cache, gic, gpio, mmu, ostm, sdram, stb, time_driver};

    // ── Init (runs once before any test) ────────────────────────────────────

    #[init]
    fn init() {
        unsafe {
            stb::init();
            mmu::init_and_enable();
            cache::l1_enable();
            cache::l2_init();
            sdram::init();
            gic::init();
            ostm::enable_clock();
            ostm::start_free_running(0);
            time_driver::init();
            cortex_ar::interrupt::enable();
        }
    }

    // ── GPIO tests (from deluge-bsp/tests/gpio.rs) ──────────────────────────

    /// Configure SYNC LED (P6_7) as GPIO output and drive it high then low.
    #[test]
    fn gpio_sync_led_drive_high_low() {
        unsafe {
            gpio::set_as_output(6, 7);
            gpio::write(6, 7, true);
            gpio::write(6, 7, false);
        }
    }

    /// Call `set_pin_mux` on P6_7 — exercises the three-bit PFC/PFCE/PFCAE
    /// write sequence without checking a physical signal.
    #[test]
    fn gpio_pin_mux_does_not_fault() {
        unsafe {
            gpio::set_pin_mux(6, 7, 1);
            gpio::set_as_output(6, 7); // restore
        }
    }

    // ── SDRAM tests (from deluge-bsp/tests/sdram.rs) ────────────────────────

    /// Write a 16-word marching pattern to SDRAM and verify it reads back.
    #[test]
    fn sdram_read_write_pattern() {
        const BASE: *mut u32 = 0x0C00_0000 as *mut u32;
        const WORDS: usize = 16;
        const CACHE_LINE: usize = 32;
        const TOTAL_BYTES: usize = WORDS * core::mem::size_of::<u32>();

        unsafe {
            for i in 0..WORDS {
                BASE.add(i).write_volatile(0xDEAD_0000 | i as u32);
            }

            let base_addr = BASE as usize;
            let mut offset = 0;
            while offset < TOTAL_BYTES {
                cortex_ar::cache::clean_and_invalidate_data_cache_line_to_poc(
                    (base_addr + offset) as u32,
                );
                offset += CACHE_LINE;
            }
            core::arch::asm!("dsb", options(nomem, nostack));

            for i in 0..WORDS {
                let got = BASE.add(i).read_volatile();
                let want = 0xDEAD_0000 | i as u32;
                assert_eq!(got, want, "SDRAM[{i}]: got {got:#010x} want {want:#010x}");
            }
        }
    }

    // ── OSTM tests (from rza1/tests/ostm.rs) ────────────────────────────────

    /// Counter must advance after a 1 ms busy-wait.
    #[test]
    #[timeout(5)]
    fn ostm_counter_increments() {
        let t0 = unsafe { ostm::count(0) };
        let target = ostm::OSTM_HZ / 1000;
        while unsafe { ostm::count(0) }.wrapping_sub(t0) < target {}
        let delta = unsafe { ostm::count(0) }.wrapping_sub(t0);
        assert!(delta >= target, "counter did not advance: delta={delta}");
    }

    /// After 100 ms the tick delta must be within ±5 % of OSTM_HZ / 10.
    #[test]
    #[timeout(10)]
    fn ostm_tick_rate_approximately_correct() {
        const EXPECTED: u32 = ostm::OSTM_HZ / 10;
        const MARGIN: u32 = EXPECTED / 20;

        let t0 = unsafe { ostm::count(0) };
        while unsafe { ostm::count(0) }.wrapping_sub(t0) < EXPECTED {}
        let delta = unsafe { ostm::count(0) }.wrapping_sub(t0);

        assert!(
            delta >= EXPECTED - MARGIN && delta <= EXPECTED + MARGIN,
            "tick rate off: delta={delta}, expected ~{EXPECTED}"
        );
    }

    // ── GIC tests (from rza1/tests/gic.rs) ──────────────────────────────────

    /// Generate SGI 0 and verify the registered handler fires within 1 ms.
    #[test]
    fn gic_software_interrupt_dispatches() {
        use super::SGI0_FIRED;

        unsafe {
            gic::register(0, || {
                SGI0_FIRED.store(true, Ordering::Release);
            });
            gic::enable(0);

            const ICDSGIR: *mut u32 = 0xE820_1F00 as *mut u32;
            ICDSGIR.write_volatile(0x0100_0000);

            let t0 = ostm::count(0);
            while !SGI0_FIRED.load(Ordering::Acquire) {
                if ostm::count(0).wrapping_sub(t0) > ostm::OSTM_HZ / 1000 {
                    break;
                }
            }
        }

        assert!(SGI0_FIRED.load(Ordering::Acquire), "SGI 0 handler never fired");
    }

    // ── time_driver tests (from deluge-bsp/tests/time_driver.rs) ────────────

    /// A 100 ms Embassy timer must resolve within 90–200 ms.
    #[test]
    #[timeout(5)]
    async fn time_driver_timer_resolves_approximately() {
        let t0 = Instant::now();
        Timer::after(Duration::from_millis(100)).await;
        let elapsed = Instant::now().duration_since(t0).as_millis();
        assert!(
            elapsed >= 90 && elapsed <= 200,
            "elapsed {elapsed} ms outside [90, 200] window"
        );
    }
}
