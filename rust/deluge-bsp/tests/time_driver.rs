//! On-device async time-driver test.
//!
//! Schedules a 100 ms Embassy timer and asserts the elapsed wall time is
//! within 90–200 ms (generous range to handle cold-boot jitter on first run).
//!
//! Requires full init chain: STB → GIC → OSTM clocks → time_driver → IRQ on.
//!
//! Run on-device: `cargo test-fw -p deluge-bsp --test time_driver`
#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
#![cfg_attr(target_os = "none", feature(impl_trait_in_assoc_type))]

#[cfg(not(target_os = "none"))]
fn main() {}

#[cfg(target_os = "none")]
use deluge_test as _;

#[cfg(target_os = "none")]
#[deluge_test::tests(executor = embassy_executor::Executor::new())]
mod tests {
    use embassy_time::{Duration, Instant, Timer};
    use rza1::{gic, ostm, stb, time_driver};

    #[init]
    fn init() {
        unsafe {
            stb::init();
            gic::init();
            ostm::enable_clock();
            time_driver::init();
            cortex_ar::interrupt::enable();
        }
    }

    /// A 100 ms timer must resolve and the measured elapsed time must be
    /// between 90 ms and 200 ms.
    #[test]
    #[timeout(5)]
    async fn timer_resolves_approximately() {
        let t0 = Instant::now();
        Timer::after(Duration::from_millis(100)).await;
        let elapsed = Instant::now().duration_since(t0).as_millis();
        assert!(
            elapsed >= 90 && elapsed <= 200,
            "elapsed {elapsed} ms outside [90, 200] window"
        );
    }
}
