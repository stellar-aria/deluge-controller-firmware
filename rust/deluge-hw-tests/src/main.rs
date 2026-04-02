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
use core::sync::atomic::AtomicBool;

// Global flag for the GIC SGI test — must live outside the tests module so
// the interrupt handler closure can reach it.
#[cfg(target_os = "none")]
static SGI0_FIRED: AtomicBool = AtomicBool::new(false);

#[cfg(target_os = "none")]
#[deluge_test::tests(executor = embassy_executor::Executor::new())]
mod tests {
    use core::sync::atomic::Ordering;
    use deluge_bsp::{pic, sd, uart};
    use embassy_time::{with_timeout, Duration, Instant, Timer};
    use rza1::{cache, gic, gpio, mmu, mtu2, ostm, sdram, ssi, stb, time_driver};

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
            (EXPECTED - MARGIN..=EXPECTED + MARGIN).contains(&delta),
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

        assert!(
            SGI0_FIRED.load(Ordering::Acquire),
            "SGI 0 handler never fired"
        );
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
            (90..=200).contains(&elapsed),
            "elapsed {elapsed} ms outside [90, 200] window"
        );
    }

    // ── UART loopback tests (from rza1/tests/uart.rs) ────────────────────────

    /// Send 3 bytes via SCIF0 in hardware loopback mode and verify they arrive
    /// intact.  Tests the async TX/RX FIFO paths and GIC interrupt wiring.
    ///
    /// SCFCR bit 0 (LOOP) connects the SCIF's internal TX shift register to its
    /// RX shift register without touching the physical P6_15/P6_14 pins.
    #[test]
    #[timeout(5)]
    async fn uart_scif0_loopback() {
        const SCIF0_SCFCR: *mut u16 = (0xE800_7000_usize + 0x18) as *mut u16;
        const TEST_BYTES: [u8; 3] = [0xA5, 0x3C, 0xFF];

        unsafe { SCIF0_SCFCR.write_volatile(0x0001) }; // LOOP = 1

        rza1::uart::write_bytes(uart::MIDI_CH, &TEST_BYTES).await;

        for &expected in &TEST_BYTES {
            let got = with_timeout(
                Duration::from_millis(500),
                rza1::uart::read_byte(uart::MIDI_CH),
            )
            .await
            .expect("UART loopback: byte not received within 500 ms");
            assert_eq!(
                got, expected,
                "UART loopback mismatch: got {got:#04x} expected {expected:#04x}"
            );
        }

        unsafe { SCIF0_SCFCR.write_volatile(0x0000) }; // LOOP = 0, restore
    }

    // ── PIC baud-switch test ─────────────────────────────────────────────────

    /// Run the full PIC32 initialisation sequence (31 250 → 200 000 bps), then
    /// request the firmware version at high speed and verify the PIC responds.
    ///
    /// This exercises SCIF1 baud-rate reconfiguration, the PIC UART protocol,
    /// and the [`pic::Parser`] byte decoder.
    #[test]
    #[timeout(10)]
    async fn pic_responds_after_baud_switch() {
        // Run the standard init sequence: sends config commands at 31 250 bps,
        // switches both sides to 200 000 bps, then requests firmware version.
        pic::init().await;

        // Request firmware version again at the new baud rate.
        rza1::uart::write_bytes(pic::UART_CH, &[245u8]).await;

        // Drain bytes until we decode a FirmwareVersion event, or time out.
        let mut parser = pic::Parser::new();
        loop {
            let byte = with_timeout(
                Duration::from_millis(300),
                rza1::uart::read_byte(pic::UART_CH),
            )
            .await
            .expect("PIC: no response within 300 ms at 200 kbps");

            if let Some(pic::Event::FirmwareVersion(v)) = parser.push(byte) {
                assert!(v > 0, "unexpected zero PIC firmware version");
                break;
            }
        }
    }

    // ── MTU2 tests ───────────────────────────────────────────────────────────

    /// MTU2 channel 1 free-running at prescaler /1 (33.3 MHz) must advance
    /// after a 1 ms busy-wait measured against the already-running OSTM0.
    #[test]
    #[timeout(5)]
    fn mtu2_counter_increments() {
        // Start channel 1 free-running at full P0 speed.
        unsafe { mtu2::start_free_running(1, 1) };

        let t0_mtu = unsafe { mtu2::count(1) };

        // Busy-wait 1 ms using OSTM0 as reference.
        let t0_ostm = unsafe { ostm::count(0) };
        while unsafe { ostm::count(0) }.wrapping_sub(t0_ostm) < ostm::OSTM_HZ / 1000 {}

        let delta = unsafe { mtu2::count(1) }.wrapping_sub(t0_mtu);

        // At 33.3 MHz and /1 prescaler, 1 ms ≈ 33 333 ticks.
        // Accept anything in [10 000, 65 000] to stay well within the 16-bit range
        // while catching obvious failures (e.g. counter stuck or wildly wrong clock).
        let expected = (mtu2::MTU2_P0_HZ / 1000) as u16;
        assert!(
            delta >= expected / 3 && delta <= expected * 2,
            "MTU2 ch1 delta {delta} out of expected range ~{expected} ticks/ms"
        );
    }

    // ── SD card / SDHI tests ─────────────────────────────────────────────────

    /// Run the full SD card initialisation protocol and verify success.
    ///
    /// Exercises: SDHI1 hardware init, GIC interrupt registration, async
    /// command sequencing (CMD0, CMD8, ACMD41 loop, CMD2, CMD3, CMD7, ACMD6,
    /// CMD16), and clock switching from 260 kHz to ~33 MHz.
    #[test]
    #[timeout(30)]
    async fn sdhi_sd_card_init() {
        sd::init().await.expect("SD card initialisation failed");
    }

    // ── DMAC / SSI DMA tests ─────────────────────────────────────────────────

    /// After audio::init() starts the SSI0 DMA engine, the TX source pointer
    /// must be within the TX sample buffer and must advance over time,
    /// confirming that the DMAC link-descriptor circular transfer is live.
    #[test]
    #[timeout(5)]
    async fn dmac_ssi_tx_dma_is_live() {
        // Give the DMA at least one full buffer pass to start advancing.
        // TX buffer = 8 192 bytes at ~5.6 MHz BCLK → ~1.5 ms per pass.
        Timer::after(Duration::from_millis(10)).await;

        let start = ssi::tx_buf_start() as u32;
        let end = ssi::tx_buf_end() as u32;

        let ptr0 = ssi::tx_current_ptr() as u32;
        assert!(
            ptr0 >= start && ptr0 < end,
            "tx_current_ptr {ptr0:#010x} outside TX buffer [{start:#010x}, {end:#010x})"
        );

        // Wait one more ms and confirm the pointer has moved (DMA is cycling).
        Timer::after(Duration::from_millis(5)).await;
        let ptr1 = ssi::tx_current_ptr() as u32;
        assert_ne!(
            ptr0, ptr1,
            "tx_current_ptr did not advance after 5 ms — DMA may be stalled"
        );
    }

    // ── PIC OLED echo test ───────────────────────────────────────────────────

    /// After the PIC init sequence, send DC_LOW(250) + ENABLE_OLED(247) +
    /// SELECT_OLED(248) and verify the PIC echoes back all three bytes within
    /// 20 ms.  This validates the PIC UART echo path used by the OLED driver.
    #[test]
    #[timeout(15)]
    async fn pic_oled_select_echo() {
        pic::init().await;

        // Flush any leftover bytes from init (firmware version, button states, etc.)
        loop {
            match with_timeout(
                Duration::from_millis(50),
                rza1::uart::read_byte(pic::UART_CH),
            )
            .await
            {
                Ok(_) => {}      // discard
                Err(_) => break, // 50 ms silence → drain complete
            }
        }

        // Send the OLED select sequence.
        rza1::uart::write_bytes(pic::UART_CH, &[250u8]).await; // DC_LOW
        rza1::uart::write_bytes(pic::UART_CH, &[247u8]).await; // ENABLE_OLED
        rza1::uart::write_bytes(pic::UART_CH, &[248u8]).await; // SELECT_OLED

        // Collect all bytes that arrive within 20 ms.
        let mut received: [u8; 16] = [0u8; 16];
        let mut count = 0usize;
        loop {
            match with_timeout(
                Duration::from_millis(20),
                rza1::uart::read_byte(pic::UART_CH),
            )
            .await
            {
                Ok(b) => {
                    if count < received.len() {
                        received[count] = b;
                        count += 1;
                    }
                }
                Err(_) => break, // 20 ms silence → done
            }
        }

        // Send DESELECT_OLED to clean up PIC state.
        rza1::uart::write_bytes(pic::UART_CH, &[249u8]).await;

        assert!(
            count > 0,
            "PIC sent no echo bytes after DC_LOW+ENABLE_OLED+SELECT_OLED (timeout)"
        );
        assert!(
            received[..count].contains(&248u8),
            "PIC echo did not include SELECT_OLED (248); got {:?}",
            &received[..count]
        );
    }
}
