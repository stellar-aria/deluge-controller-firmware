//! On-device GPIO smoke tests.
//!
//! Verifies that configuring a GPIO pin as output and driving it does not
//! cause a fault or panic. Exercises `set_as_output`, `write`, and
//! `set_pin_mux` on the SYNC LED pin (P6_7).
//!
//! Run on-device: `cargo test-fw -p deluge-bsp --test gpio`
#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(not(target_os = "none"))]
fn main() {}

#[cfg(target_os = "none")]
use deluge_test as _;

#[cfg(target_os = "none")]
#[deluge_test::tests]
mod tests {
    use rza1::{gpio, stb};

    #[init]
    fn init() {
        unsafe {
            stb::init();
        }
    }

    /// Configure SYNC LED (P6_7) as GPIO output and drive it high then low.
    /// Smoke test: verifies no data-abort or panic occurs.
    #[test]
    fn sync_led_drive_high_low() {
        unsafe {
            gpio::set_as_output(6, 7);
            gpio::write(6, 7, true);
            gpio::write(6, 7, false);
        }
    }

    /// Call `set_pin_mux` on P6_7 — exercises the three-bit PFC/PFCE/PFCAE
    /// write sequence without checking a physical signal.
    #[test]
    fn pin_mux_does_not_fault() {
        unsafe {
            gpio::set_pin_mux(6, 7, 1);
            gpio::set_as_output(6, 7); // restore
        }
    }
}
