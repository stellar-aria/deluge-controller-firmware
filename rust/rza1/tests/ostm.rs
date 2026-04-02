//! On-device integration tests for the OSTM (OS Timer) driver.
//!
//! Loaded by probe-rs directly into OCRAM; startup is provided by
//! `deluge_bsp::startup`. Two test cases:
//!   - `counter_increments`      — sanity-check the counter is running
//!   - `tick_rate_approximately_correct` — 100 ms bucket check against OSTM_HZ
//!
//! Run on-device: `cargo test-fw -p rza1 --test ostm`
#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(not(target_os = "none"))]
fn main() {}

#[cfg(target_os = "none")]
use deluge_test as _;

#[cfg(target_os = "none")]
#[deluge_test::tests]
mod tests {
    use rza1::{ostm, stb};

    #[init]
    fn init() {
        unsafe {
            stb::init();
            ostm::enable_clock();

            // Diagnostic: print OSTM0 register state before and after init
            let base = 0xFCFEC000usize;
            let ctl_before = ((base + 0x20) as *const u8).read_volatile();
            let te_before = ((base + 0x10) as *const u8).read_volatile();
            let cnt_before = ((base + 0x04) as *const u32).read_volatile();
            rtt_target::rprintln!(
                "[ostm init] before start: CTL={:#04x} TE={} CNT={:#010x}",
                ctl_before,
                te_before,
                cnt_before
            );

            ostm::start_free_running(0);

            let ctl_after = ((base + 0x20) as *const u8).read_volatile();
            let te_after = ((base + 0x10) as *const u8).read_volatile();
            let cnt_after = ((base + 0x04) as *const u32).read_volatile();
            rtt_target::rprintln!(
                "[ostm init] after  start: CTL={:#04x} TE={} CNT={:#010x}",
                ctl_after,
                te_after,
                cnt_after
            );
        }
    }

    /// Counter must advance after a 1 ms busy-wait.
    #[test]
    #[timeout(5)]
    fn counter_increments() {
        let t0 = unsafe { ostm::count(0) };
        let t1 = unsafe { ostm::count(0) };
        let t2 = unsafe { ostm::count(0) };
        rtt_target::rprintln!(
            "[counter_increments] t0={:#010x} t1={:#010x} t2={:#010x}",
            t0,
            t1,
            t2
        );
        // busy-wait ~1 ms (OSTM_HZ / 1000 ticks)
        let target = ostm::OSTM_HZ / 1000;
        while unsafe { ostm::count(0) }.wrapping_sub(t0) < target {}
        let delta = unsafe { ostm::count(0) }.wrapping_sub(t0);
        assert!(delta >= target, "counter did not advance: delta={delta}");
    }

    /// After 100 ms the tick delta must be within ±5 % of OSTM_HZ / 10.
    #[test]
    #[timeout(10)]
    fn tick_rate_approximately_correct() {
        const EXPECTED: u32 = ostm::OSTM_HZ / 10; // 3_333_333 ticks
        const MARGIN: u32 = EXPECTED / 20; // 5 %

        let t0 = unsafe { ostm::count(0) };
        while unsafe { ostm::count(0) }.wrapping_sub(t0) < EXPECTED {}
        let delta = unsafe { ostm::count(0) }.wrapping_sub(t0);

        assert!(
            delta >= EXPECTED - MARGIN && delta <= EXPECTED + MARGIN,
            "tick rate off: delta={delta}, expected ~{EXPECTED}"
        );
    }
}
