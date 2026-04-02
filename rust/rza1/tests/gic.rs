//! On-device integration tests for the GIC (Generic Interrupt Controller).
//!
//! Uses SGI (Software-Generated Interrupt) 0 to verify the full dispatch path:
//! gic_dispatch → handler → AtomicBool flag — without needing a real device
//! interrupt source.
//!
//! Run on-device: `cargo test-fw -p rza1 --test gic`
#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(not(target_os = "none"))]
fn main() {}

#[cfg(target_os = "none")]
use deluge_test as _;

#[cfg(target_os = "none")]
use core::sync::atomic::{AtomicBool, Ordering};

#[cfg(target_os = "none")]
static SGI0_FIRED: AtomicBool = AtomicBool::new(false);

#[cfg(target_os = "none")]
#[deluge_test::tests]
mod tests {
    use core::sync::atomic::Ordering;
    use rza1::{gic, ostm, stb};

    #[init]
    fn init() {
        unsafe {
            stb::init();
            ostm::enable_clock();
            ostm::start_free_running(0);
            gic::init();
        }
    }

    /// Write ICDSGIR to generate SGI 0 targeting CPU 0; the registered handler
    /// sets `SGI0_FIRED`. Verify the flag is set within a bounded spin.
    #[test]
    fn software_interrupt_dispatches() {
        use super::SGI0_FIRED;

        unsafe {
            gic::register(0, || {
                SGI0_FIRED.store(true, Ordering::Release);
            });
            gic::enable(0);
            cortex_ar::interrupt::enable();

            // ICDSGIR: TargetListFilter=01 (self), IntID=0
            const ICDSGIR: *mut u32 = 0xE820_1F00 as *mut u32;
            ICDSGIR.write_volatile(0x0100_0000);

            // spin up to ~1 ms
            let t0 = ostm::count(0);
            while !SGI0_FIRED.load(Ordering::Acquire) {
                if ostm::count(0).wrapping_sub(t0) > ostm::OSTM_HZ / 1000 {
                    break;
                }
            }

            cortex_ar::interrupt::disable();
        }

        assert!(
            SGI0_FIRED.load(Ordering::Acquire),
            "SGI 0 handler never fired"
        );
    }
}
