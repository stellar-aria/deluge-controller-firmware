//! Async SCIF UART driver for RZ/A1L.
//!
//! Supports all five SCIF channels available on RZ/A1L (SCIF0–SCIF4).
//! Base addresses (stride 0x800):
//!   SCIF0 @ 0xE800_7000 … SCIF4 @ 0xE800_9000
//!
//! GIC interrupt IDs (per RZ/A1L HW Manual §A.2, devdrv_intc.h):
//!   SCIF_n: BRI = 221+n*4, ERI = 222+n*4, RXI = 223+n*4, TXI = 224+n*4
//!
//! **This module is board-agnostic.** It does not know which ports are wired
//! to which SCIF channel. Configure GPIO pin-mux via [`crate::gpio::set_pin_mux`]
//! before (or after) calling [`init`].
//!
//! ## Usage
//! ```ignore
//! unsafe {
//!     // 1. Init the SCIF peripheral
//!     uart::init(0, 31250);
//!     // 2. Configure GPIO pins (board-specific)
//!     gpio::set_pin_mux(6, 15, 5); // TxD0 on Deluge
//!     gpio::set_pin_mux(6, 14, 5); // RxD0 on Deluge
//!     // 3. Register GIC IRQs for this channel
//!     uart::register_irqs_for(0);
//! }
//! // In an Embassy task:
//! uart::write_bytes(0, b"hello").await;
//! let b = uart::read_byte(0).await;
//! ```

use core::future::poll_fn;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use crate::gic;

// ---------------------------------------------------------------------------
// SCIF register offsets (see RZ/A1L HW Manual §24, struct st_scif layout)
// ---------------------------------------------------------------------------

const SCSMR:  usize = 0x00; // u16  Serial Mode Register
const SCBRR:  usize = 0x04; // u8   Bit Rate Register
const SCSCR:  usize = 0x08; // u16  Serial Control Register
const FTDR:   usize = 0x0C; // u8   FIFO Transmit Data Register (write-only)
const SCFSR:  usize = 0x10; // u16  FIFO Status Register
const SCFRDR: usize = 0x14; // u8   FIFO Receive Data Register (read-only)
const SCFCR:  usize = 0x18; // u16  FIFO Control Register
const SCFDR:  usize = 0x1C; // u16  FIFO Data Count Register  [12:8]=TX  [4:0]=RX
const SCSPTR: usize = 0x20; // u16  Serial Port Register
const SCLSR:  usize = 0x24; // u16  Line Status Register
const SCEMR:  usize = 0x28; // u16  Serial Extended Mode Register

// SCSCR bit masks
const TE:  u16 = 1 << 4; // Transmit Enable
const RE:  u16 = 1 << 5; // Receive Enable
const RIE: u16 = 1 << 6; // Receive Interrupt Enable  (enables RXI + ERI)
const TIE: u16 = 1 << 7; // Transmit Interrupt Enable (enables TXI)

// SCFSR bit masks
const DR:   u16 = 1 << 0; // Data Ready  (RX FIFO has data below trigger)
const RDF:  u16 = 1 << 1; // Receive FIFO Data Full  (count ≥ trigger)
const TDFE: u16 = 1 << 5; // TX FIFO Data Empty  (count ≤ trigger)

// TX FIFO depth
const TX_FIFO_SIZE: usize = 16;

// Peripheral clock: XTAL(≈13.2 MHz) × 5 ≈ 66.128 MHz.
// Used for SCBRR calculation with BGDM=1 (double-speed mode):
//   SCBRR = round(P_CLK / (16 × baud)) − 1
const P_CLK: u32 = 66_128_125;

/// Number of SCIF channels on RZ/A1L.
pub const NUM_CHANNELS: usize = 5;

// GIC base IDs for SCIF0; pattern: RXI_n = RXI_BASE + n*4, TXI_n = TXI_BASE + n*4
const RXI_BASE: u16 = 223;
const TXI_BASE: u16 = 224;

// GIC interrupt priority for UART (lower number = higher priority).
const UART_IRQ_PRIORITY: u8 = 10;

// ---------------------------------------------------------------------------
// Per-channel async state
// ---------------------------------------------------------------------------

struct UartState {
    rx_waker: AtomicWaker,
    tx_waker: AtomicWaker,
}

// Safety: AtomicWaker is Send + Sync.
unsafe impl Sync for UartState {}

static UART_STATE: [UartState; NUM_CHANNELS] = [
    UartState { rx_waker: AtomicWaker::new(), tx_waker: AtomicWaker::new() },
    UartState { rx_waker: AtomicWaker::new(), tx_waker: AtomicWaker::new() },
    UartState { rx_waker: AtomicWaker::new(), tx_waker: AtomicWaker::new() },
    UartState { rx_waker: AtomicWaker::new(), tx_waker: AtomicWaker::new() },
    UartState { rx_waker: AtomicWaker::new(), tx_waker: AtomicWaker::new() },
];

// ---------------------------------------------------------------------------
// Register helpers
// ---------------------------------------------------------------------------

#[inline(always)]
unsafe fn rr16(addr: usize) -> u16 {
    core::ptr::read_volatile(addr as *const u16)
}

#[inline(always)]
unsafe fn wr16(addr: usize, val: u16) {
    core::ptr::write_volatile(addr as *mut u16, val);
}

#[inline(always)]
unsafe fn rr8(addr: usize) -> u8 {
    core::ptr::read_volatile(addr as *const u8)
}

#[inline(always)]
unsafe fn wr8(addr: usize, val: u8) {
    core::ptr::write_volatile(addr as *mut u8, val);
}

/// Base address of SCIF channel `ch`.
#[inline(always)]
fn base(ch: usize) -> usize {
    0xE800_7000 + ch * 0x0800
}

// ---------------------------------------------------------------------------
// Baud rate calculation
// ---------------------------------------------------------------------------

/// Compute SCBRR value for the given baud rate.
///
/// With BGDM=1 (double-speed mode): SCBRR = round(P_CLK / (16 × baud)) − 1.
/// Integer rounding: (P_CLK + 8 × baud) / (16 × baud) − 1.
fn scbrr(baud: u32) -> u8 {
    (((P_CLK + 8 * baud) / (16 * baud)) - 1) as u8
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

/// Initialise SCIF channel `ch` (`0`–`4`) at `baud_rate` bps.
///
/// This function only programmes the SCIF peripheral registers. GPIO pin-mux
/// for the TX/RX pins must be configured separately via
/// [`crate::gpio::set_pin_mux`] using board-specific port/pin/mux values.
///
/// Must be called before [`register_irqs_for`] and with global IRQ disabled.
///
/// # Safety
/// Writes to memory-mapped SCIF registers. Caller must ensure `ch` < [`NUM_CHANNELS`].
pub unsafe fn init(ch: usize, baud_rate: u32) {
    debug_assert!(ch < NUM_CHANNELS);
    let b = base(ch);

    // 1. Stop all SCIF operations.
    wr16(b + SCSCR, 0x0000);

    // 2. Reset TX and RX FIFOs (TFRST=bit2, RFRST=bit1).
    wr16(b + SCFCR, 0x0006);

    // 3. Clear status flags: mask out reserved/sticky bits, keep ER/BRK/DR clear.
    let scfsr = rr16(b + SCFSR);
    wr16(b + SCFSR, scfsr & 0xFF6E);

    // 4. Clear overrun error in SCLSR (ORER = bit 0).
    let sclsr = rr16(b + SCLSR);
    wr16(b + SCLSR, sclsr & !1u16);

    // 5. Internal clock source: CKE[1:0] = 00 (already 0 from step 1).

    // 6. Async 8N1, CKS=00 (P_CLK/1).
    wr16(b + SCSMR, 0x0000);

    // 7. Double-speed mode: BGDM = bit 7.
    wr16(b + SCEMR, 0x0080);

    // 8. Baud rate.
    wr8(b + SCBRR, scbrr(baud_rate));

    // 9. Release FIFO resets.  TX trigger=8 (TTRG=00), RX trigger=1 (RTRG=00).
    wr16(b + SCFCR, 0x0000);

    // 10. Break: keep TxD high.
    let scsptr = rr16(b + SCSPTR);
    wr16(b + SCSPTR, scsptr | 0x0003);

    // 11. Enable TX and RX; leave TIE/RIE off (enabled on demand by futures).
    wr16(b + SCSCR, TE | RE);
}

// ---------------------------------------------------------------------------
// GIC interrupt registration
// ---------------------------------------------------------------------------

/// Change the baud rate of an already-initialised SCIF channel without
/// resetting the FIFOs or GIC interrupt registration.
///
/// The SCIF must briefly stop TX/RX while SCBRR is updated.  Any bytes
/// currently being shifted out will be lost; the caller must ensure the TX
/// FIFO has drained (e.g. via `Timer::after_millis`) before calling this.
///
/// # Safety
/// Writes to memory-mapped SCIF registers. Caller must ensure `ch` < [`NUM_CHANNELS`].
pub unsafe fn set_baud(ch: usize, baud_rate: u32) {
    debug_assert!(ch < NUM_CHANNELS);
    let b = base(ch);
    // Stop TE/RE so the baud rate change takes effect cleanly.
    wr16(b + SCSCR, 0x0000);
    wr8(b + SCBRR, scbrr(baud_rate));
    // Re-enable TX and RX (leave TIE/RIE off; they are set on demand by futures).
    wr16(b + SCSCR, TE | RE);
}

/// Register and enable the RXI and TXI GIC interrupt sources for SCIF
/// channel `ch`.
///
/// Must be called after [`init`] and before global IRQ is enabled.
///
/// # Safety
/// Writes to memory-mapped GIC registers via [`crate::gic`].
/// Caller must ensure `ch` < [`NUM_CHANNELS`].
pub unsafe fn register_irqs_for(ch: usize) {
    debug_assert!(ch < NUM_CHANNELS);
    let rxi = RXI_BASE + (ch as u16) * 4;
    let txi = TXI_BASE + (ch as u16) * 4;
    gic::register(rxi, RXI_HANDLERS[ch]);
    gic::register(txi, TXI_HANDLERS[ch]);
    gic::set_priority(rxi, UART_IRQ_PRIORITY);
    gic::set_priority(txi, UART_IRQ_PRIORITY);
    gic::enable(rxi);
    gic::enable(txi);
}

// ---------------------------------------------------------------------------
// Interrupt handlers
// ---------------------------------------------------------------------------

/// Called from GIC dispatch when RXI fires for SCIF `ch`.
/// Disables RIE (stops further RXI) and wakes any pending rx future.
fn on_rxi(ch: usize) {
    let b = base(ch);
    unsafe {
        let scscr = rr16(b + SCSCR);
        wr16(b + SCSCR, scscr & !RIE);
    }
    UART_STATE[ch].rx_waker.wake();
}

/// Called from GIC dispatch when TXI fires for SCIF `ch`.
/// Disables TIE (stops further TXI) and wakes any pending tx future.
fn on_txi(ch: usize) {
    let b = base(ch);
    unsafe {
        let scscr = rr16(b + SCSCR);
        wr16(b + SCSCR, scscr & !TIE);
    }
    UART_STATE[ch].tx_waker.wake();
}

fn rxi0_handler() { on_rxi(0); }
fn txi0_handler() { on_txi(0); }
fn rxi1_handler() { on_rxi(1); }
fn txi1_handler() { on_txi(1); }
fn rxi2_handler() { on_rxi(2); }
fn txi2_handler() { on_txi(2); }
fn rxi3_handler() { on_rxi(3); }
fn txi3_handler() { on_txi(3); }
fn rxi4_handler() { on_rxi(4); }
fn txi4_handler() { on_txi(4); }

type HandlerFn = fn();
static RXI_HANDLERS: [HandlerFn; NUM_CHANNELS] =
    [rxi0_handler, rxi1_handler, rxi2_handler, rxi3_handler, rxi4_handler];
static TXI_HANDLERS: [HandlerFn; NUM_CHANNELS] =
    [txi0_handler, txi1_handler, txi2_handler, txi3_handler, txi4_handler];

// ---------------------------------------------------------------------------
// Async API
// ---------------------------------------------------------------------------

/// Read one byte from the receive FIFO of SCIF channel `ch`.
///
/// Suspends the calling task if no data is available.
pub async fn read_byte(ch: usize) -> u8 {
    let b = base(ch);
    poll_fn(|cx| {
        // Register waker BEFORE checking the FIFO so we cannot miss a wakeup
        // from an ISR that fires between the check and returning Pending.
        UART_STATE[ch].rx_waker.register(cx.waker());

        let scfsr = unsafe { rr16(b + SCFSR) };
        if scfsr & (RDF | DR) != 0 {
            // Data is available: read one byte and clear the status flags.
            let byte = unsafe { rr8(b + SCFRDR) };
            // Clear RDF/DR by writing 0 to those bits (hardware re-asserts if
            // more data remains in the FIFO at or above the trigger level).
            unsafe { wr16(b + SCFSR, scfsr & !(RDF | DR)) };
            Poll::Ready(byte)
        } else {
            // Enable RIE so the ISR wakes us when data arrives.
            unsafe {
                let scscr = rr16(b + SCSCR);
                wr16(b + SCSCR, scscr | RIE);
            }
            // Re-check after enabling RIE to close the race where data arrived
            // between our initial FIFO check and the RIE write.
            let scfsr = unsafe { rr16(b + SCFSR) };
            if scfsr & (RDF | DR) != 0 {
                // Data arrived in the window — cancel the interrupt and return Ready.
                unsafe {
                    let scscr = rr16(b + SCSCR);
                    wr16(b + SCSCR, scscr & !RIE);
                    let byte = rr8(b + SCFRDR);
                    wr16(b + SCFSR, scfsr & !(RDF | DR));
                    return Poll::Ready(byte);
                }
            }
            Poll::Pending
        }
    })
    .await
}

/// Write `buf` to the transmit FIFO of SCIF channel `ch`.
///
/// Suspends the calling task whenever the TX FIFO is full (≥16 bytes), then
/// resumes when the FIFO has drained to the trigger level (≤8 bytes) as
/// signalled by the TXI interrupt.
pub async fn write_bytes(ch: usize, buf: &[u8]) {
    let b = base(ch);
    let mut pos = 0;

    while pos < buf.len() {
        // How many bytes can we add right now?
        let in_fifo = unsafe { (rr16(b + SCFDR) >> 8) & 0x1F } as usize;
        let space = TX_FIFO_SIZE.saturating_sub(in_fifo);
        let to_write = (buf.len() - pos).min(space);

        for i in 0..to_write {
            unsafe { wr8(b + FTDR, buf[pos + i]) };
        }
        pos += to_write;

        if pos < buf.len() {
            // FIFO is full; wait for TDFE (count ≤ trigger) via TXI interrupt.
            wait_tdfe(ch).await;
        }
    }
}

/// Suspend until the TX FIFO has drained below the trigger level (TDFE=1).
async fn wait_tdfe(ch: usize) {
    let b = base(ch);
    poll_fn(|cx| {
        UART_STATE[ch].tx_waker.register(cx.waker());

        let scfsr = unsafe { rr16(b + SCFSR) };
        if scfsr & TDFE != 0 {
            Poll::Ready(())
        } else {
            // Enable TIE so the ISR fires when TDFE goes high.
            unsafe {
                let scscr = rr16(b + SCSCR);
                wr16(b + SCSCR, scscr | TIE);
            }
            // Double-check after enabling TIE to close the race.
            let scfsr = unsafe { rr16(b + SCFSR) };
            if scfsr & TDFE != 0 {
                unsafe {
                    let scscr = rr16(b + SCSCR);
                    wr16(b + SCSCR, scscr & !TIE);
                }
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    })
    .await
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::{base, scbrr, NUM_CHANNELS, P_CLK, RXI_BASE, TXI_BASE};

    // SCIF0 base = 0xE800_7000, stride 0x800 per channel.
    #[test]
    fn scif_base_addresses() {
        assert_eq!(base(0), 0xE800_7000);
        assert_eq!(base(1), 0xE800_7800);
        assert_eq!(base(2), 0xE800_8000);
        assert_eq!(base(3), 0xE800_8800);
        assert_eq!(base(4), 0xE800_9000);
    }

    #[test]
    fn num_channels_is_five() {
        assert_eq!(NUM_CHANNELS, 5);
    }

    /// MIDI baud rate (31250 bps).
    ///
    /// SCBRR = round(P_CLK / (16 × 31250)) − 1
    ///       = round(66128125 / 500000) − 1
    ///       = round(132.256) − 1 = 132 − 1 = 131
    #[test]
    fn scbrr_midi_31250() {
        assert_eq!(scbrr(31_250), 131);
    }

    /// 115200 bps (debug serial).
    ///
    /// SCBRR = round(66128125 / 1843200) − 1 = round(35.88) − 1 = 36 − 1 = 35
    #[test]
    fn scbrr_115200() {
        assert_eq!(scbrr(115_200), 35);
    }

    /// Spot-check that the resulting baud rate error is within ±2% for MIDI.
    #[test]
    fn scbrr_midi_accuracy() {
        let divisor = scbrr(31_250) as u32 + 1;
        let actual_baud = P_CLK / (16 * divisor);
        let error_ppm = (actual_baud as i64 - 31_250) * 1_000_000 / 31_250;
        assert!(error_ppm.abs() < 20_000, "MIDI baud error {error_ppm} ppm exceeds 2%");
    }

    /// GIC interrupt IDs follow the pattern: RXI_n = RXI_BASE + n*4.
    /// RZ/A1L HW Manual §A.2: SCIF0 RXI=223, SCIF1 RXI=227, ...
    #[test]
    fn scif_irq_ids() {
        for ch in 0..NUM_CHANNELS as u16 {
            assert_eq!(RXI_BASE + ch * 4, 223 + ch * 4);
            assert_eq!(TXI_BASE + ch * 4, 224 + ch * 4);
        }
    }
}
