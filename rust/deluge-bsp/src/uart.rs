//! Deluge board UART assignments.
//!
//! Maps the two SCIF channels used on the Deluge to their physical pins and
//! gives them meaningful names, keeping [`rza1::uart`] itself board-agnostic.
//!
//! | Channel | Signal  | TxD pin  | mux | RxD pin | mux |
//! |---------|---------|----------|-----|---------|-----|
//! | SCIF0   | MIDI DIN| P6_15    |  5  | P6_14   |  5  |
//! | SCIF1   | PIC32   | P3_15    |  5  | P1_9    |  3  |

/// SCIF channel used for MIDI DIN (31250 bps).
pub const MIDI_CH: usize = 0;

/// SCIF channel used for the PIC32 co-processor.
pub const PIC_CH: usize = 1;

pub use rza1::uart::{read_byte, write_bytes};

/// Initialise the MIDI UART (SCIF0) and register its GIC interrupts.
///
/// Configures the SCIF peripheral at `baud_rate` bps, sets P6_15/P6_14 to
/// their SCIF function, then enables RXI0/TXI0 in the GIC.
///
/// # Safety
/// Must be called with global IRQs disabled, before the Embassy executor starts.
pub unsafe fn init_midi(baud_rate: u32) {
    rza1::uart::init(MIDI_CH, baud_rate);
    rza1::gpio::set_pin_mux(6, 15, 5); // TxD0
    rza1::gpio::set_pin_mux(6, 14, 5); // RxD0
    rza1::uart::register_irqs_for(MIDI_CH);
}

/// Initialise the PIC32 UART (SCIF1) and register its GIC interrupts.
///
/// Configures the SCIF peripheral at `baud_rate` bps, sets P3_15/P1_9 to
/// their SCIF function, then enables RXI1/TXI1 in the GIC.
///
/// # Safety
/// Must be called with global IRQs disabled, before the Embassy executor starts.
pub unsafe fn init_pic(baud_rate: u32) {
    rza1::uart::init(PIC_CH, baud_rate);
    rza1::gpio::set_pin_mux(3, 15, 5); // TxD1
    rza1::gpio::set_pin_mux(1, 9, 3);  // RxD1
    rza1::uart::register_irqs_for(PIC_CH);
}
