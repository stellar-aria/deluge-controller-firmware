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

/// Write bytes to the MIDI DIN UART (SCIF0 @ 31 250 bps).
#[inline]
pub async fn write_midi(data: &[u8]) {
    rza1::uart::write_bytes(MIDI_CH, data).await;
}

/// Read one byte from the MIDI DIN UART (SCIF0 DMA RX ring).
#[inline]
pub async fn read_midi_byte() -> u8 {
    rza1::uart::read_byte(MIDI_CH).await
}

/// Initialise the MIDI DIN UART (SCIF0) with DMA-backed RX.
///
/// MIDI TX uses TXI GIC interrupts; SCIF0 RX uses DMAC channel 13 so that
/// incoming bytes are captured without interrupt gaps.
///
/// DMAC channel 13, DMARS = 0x62 (SCIF0 RX resource selector):
///   DMARS_FOR_SCIF0_RX = 0b001100010 = 0x62.
///
/// # Safety
/// Must be called with global IRQs disabled, before the Embassy executor starts.
pub unsafe fn init_midi(baud_rate: u32) {
    log::debug!(
        "bsp uart: MIDI SCIF{} @ {} bps (DMA RX ch13)",
        MIDI_CH,
        baud_rate
    );
    rza1::uart::init(MIDI_CH, baud_rate);
    rza1::gpio::set_pin_mux(6, 15, 5); // TxD0
    rza1::gpio::set_pin_mux(6, 14, 5); // RxD0
    // TX via TXI interrupt; RX via DMAC channel 13.
    rza1::uart::register_txi_for(MIDI_CH);
    rza1::uart::init_dma_rx(MIDI_CH, 13, 0x62);
}

/// Initialise the PIC32 UART (SCIF1) with DMA-backed RX and TX.
///
/// SCIF1 RX uses DMAC channel 12 so that incoming bytes are captured
/// reliably without any RXI interrupt gaps.  SCIF1 TX uses DMAC channel 10
/// so that outgoing LED burst messages are sent without generating TXI
/// interrupts that would starve the USB audio task.
///
/// DMAC channel 12, DMARS = 0x66 (SCIF1 RX resource selector):
///   DMARS_FOR_SCIF0_RX = 0x62; for SCIF1: 0x62 + (1 × 4) = 0x66.
///
/// DMAC channel 10, DMARS = 0x65 (SCIF1 TX resource selector):
///   DMARS_FOR_SCIF0_TX = 0x61; for SCIF1: 0x61 + (1 × 4) = 0x65.
///
/// # Safety
/// Must be called with global IRQs disabled, before the Embassy executor starts.
pub unsafe fn init_pic(baud_rate: u32) {
    log::debug!(
        "bsp uart: PIC SCIF{} @ {} bps (DMA RX ch12, DMA TX ch10)",
        PIC_CH,
        baud_rate
    );
    rza1::uart::init(PIC_CH, baud_rate);
    rza1::gpio::set_pin_mux(3, 15, 5); // TxD1
    rza1::gpio::set_pin_mux(1, 9, 3); // RxD1
    // RX via DMAC channel 12; TX via DMAC channel 10.
    rza1::uart::register_txi_for(PIC_CH);
    rza1::uart::init_dma_rx(PIC_CH, 12, 0x66);
    rza1::uart::init_dma_tx(PIC_CH, 10, 0x65);
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::{MIDI_CH, PIC_CH};

    #[test]
    fn midi_channel_is_scif0() {
        assert_eq!(MIDI_CH, 0);
    }

    #[test]
    fn pic_channel_is_scif1() {
        assert_eq!(PIC_CH, 1);
    }

    /// Channels must be distinct so both UARTs can operate concurrently.
    #[test]
    fn channels_are_distinct() {
        assert_ne!(MIDI_CH, PIC_CH);
    }

    /// Both channels must be within the RZ/A1L SCIF range (0–4).
    #[test]
    fn channels_in_range() {
        assert!(MIDI_CH < rza1::uart::NUM_CHANNELS);
        assert!(PIC_CH < rza1::uart::NUM_CHANNELS);
    }
}
