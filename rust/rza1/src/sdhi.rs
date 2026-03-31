//! Renesas RZ/A1L SD Host Interface (SDHI) register-level driver.
//!
//! Two SDHI controllers are present on RZ/A1L:
//!   - SDHI0 at 0xE804_E000 (not used on Deluge)
//!   - SDHI1 at 0xE804_E800 (Deluge SD card — port 1)
//!
//! All registers are 16-bit, addressed at `base + (index << 1)`.
//! The data FIFO (SD_BUF0) is at `base + 0x60`, accessed as 32-bit words.
//!
//! GIC interrupt IDs for SDHI1:
//!   305 = INTC_ID_SDHI1_3  (card-detect / status-change)
//!   306 = INTC_ID_SDHI1_0  (command / data complete)
//!   307 = INTC_ID_SDHI1_1  (SDIO interrupt — registered but masked)
//!
//! ## Async model
//!
//! Each port has one [`embassy_sync::waitqueue::AtomicWaker`].  The GIC
//! handler captures INFO1/INFO2 into atomics, then wakes the waker.
//! `send_cmd`, `read_blocks_sw`, and `write_blocks_sw` use `poll_fn` to
//! sleep until the expected interrupt bits arrive.
//!
//! ## Usage
//!
//! ```ignore
//! unsafe {
//!     sdhi::init(1);
//!     sdhi::register_irqs(1);
//! }
//! // In an async task — SD card protocol:
//! sdhi::send_cmd(1, 0 /*CMD0*/, 0).await?;   // Reset
//! let resp = sdhi::last_r1(1);
//! ```

use core::future::poll_fn;
use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use crate::gic;

// ---------------------------------------------------------------------------
// Base addresses
// ---------------------------------------------------------------------------

const SDHI0_BASE: usize = 0xE804_E000;
const SDHI1_BASE: usize = 0xE804_E800;

// ---------------------------------------------------------------------------
// Register offsets (pre-multiplied by 2: index << SD_REG_SHIFT, SD_REG_SHIFT=1)
// ---------------------------------------------------------------------------

const OFF_CMD:            usize = 0x00; // SD Command
const OFF_ARG0:           usize = 0x08; // Argument low  [15:0]
const OFF_ARG1:           usize = 0x0C; // Argument high [31:16]
const OFF_STOP:           usize = 0x10; // Data Stop / Automatic Stop
const OFF_SECCNT:         usize = 0x14; // Block Count
const OFF_RESP0:          usize = 0x18; // Response R[23:8]
const OFF_RESP1:          usize = 0x1C; // Response R[39:24]
const OFF_RESP2:          usize = 0x20; // Response R[55:40]
const OFF_RESP3:          usize = 0x24; // Response R[71:56]
const OFF_RESP4:          usize = 0x28; // Response R[87:72]
const OFF_RESP5:          usize = 0x2C; // Response R[103:88]
const OFF_RESP6:          usize = 0x30; // Response R[119:104]
const OFF_RESP7:          usize = 0x34; // Response R[127:120]
const OFF_INFO1:          usize = 0x38; // Status flag 1
const OFF_INFO2:          usize = 0x3C; // Status flag 2
const OFF_INFO1_MASK:     usize = 0x40; // Interrupt mask 1  (1 = enables irq)
const OFF_INFO2_MASK:     usize = 0x44; // Interrupt mask 2  (1 = enables irq)
const OFF_CLK_CTRL:       usize = 0x48; // Clock control
const OFF_SIZE:           usize = 0x4C; // Block size
const OFF_OPTION:         usize = 0x50; // Access option (timing, bus width)
const OFF_ERR_STS1:       usize = 0x58; // CMD/CRC/END error status
const OFF_ERR_STS2:       usize = 0x5C; // Timeout error status
const OFF_BUF0:           usize = 0x60; // Data FIFO — 32-bit access
const OFF_SDIO_MODE:      usize = 0x68; // SDIO mode
const OFF_SDIO_INFO1:     usize = 0x6C; // SDIO interrupt flag
const OFF_SDIO_INFO1_MASK: usize = 0x70; // SDIO interrupt mask
const OFF_CC_EXT_MODE:    usize = 0x1B0; // DMA mode enable
const OFF_SOFT_RST:       usize = 0x1C0; // Soft reset
const OFF_EXT_SWAP:       usize = 0x1E0; // Byte-swap control

// ---------------------------------------------------------------------------
// SD_INFO1 bit masks
// ---------------------------------------------------------------------------

/// Response end (CMD complete, response received).
pub const INFO1_RESP:        u16 = 0x0001;
/// Data transfer end (all blocks transferred).
pub const INFO1_DATA_TRNS:   u16 = 0x0004;
/// Card remove via DAT3 detect.
pub const INFO1_REM_DAT3:    u16 = 0x0100;
/// Card insert  via DAT3 detect.
pub const INFO1_INS_DAT3:    u16 = 0x0200;
/// Card remove  via CD pin.
pub const INFO1_REM_CD:      u16 = 0x0008;
/// Card insert  via CD pin.
pub const INFO1_INS_CD:      u16 = 0x0010;
/// Card detect change (insert OR remove) — CD pin.
pub const INFO1_DET_CD:      u16 = 0x0018;

// ---------------------------------------------------------------------------
// SD_INFO2 bit masks
// ---------------------------------------------------------------------------

/// CMD error (CRC, end-bit, index).
pub const INFO2_ERR0:        u16 = 0x0001;
/// CRC error.
pub const INFO2_ERR1:        u16 = 0x0002;
/// End-bit error.
pub const INFO2_ERR2:        u16 = 0x0004;
/// Data timeout.
pub const INFO2_ERR3:        u16 = 0x0008;
/// Data CRC error.
pub const INFO2_ERR4:        u16 = 0x0010;
/// Response timeout.
pub const INFO2_ERR5:        u16 = 0x0020;
/// Response timeout extended.
pub const INFO2_ERR6:        u16 = 0x0040;
/// Buffer Read Enable (FIFO has data to read).
pub const INFO2_BRE:         u16 = 0x0100;
/// Buffer Write Enable (FIFO is ready to accept data).
pub const INFO2_BWE:         u16 = 0x0200;
/// Command type register busy.
pub const INFO2_CBSY:        u16 = 0x4000;
/// Illegal access (register out-of-sequence).
pub const INFO2_ILA:         u16 = 0x8000;
/// Bus clock divider change complete.
pub const INFO2_SCLKDIVEN:   u16 = 0x2000;

/// All error bits in INFO2.
pub const INFO2_ERR_ALL:     u16 = 0x807F;

// ---------------------------------------------------------------------------
// CC_EXT_MODE bits
// ---------------------------------------------------------------------------
const CC_EXT_MODE_DMASDRW: u16 = 1 << 1; // Enable SDHI DMA read/write

// ---------------------------------------------------------------------------
// Clock divider constants (input clock ≈ 66.6 MHz)
// ---------------------------------------------------------------------------

/// SD_CLK_CTRL register: divider field in bits [7:0].
/// Divider = 2^(n+1);  n=0 → ÷2, n=7 → ÷256.
/// Setting bit 8 also enables the clock output.
const CLK_DIV_256: u16 = 0x80;  // 66.6 / 256 ≈ 260 kHz (init / card-detect)
const CLK_DIV_2:   u16 = 0x01;  // 66.6 /   2 ≈  33 MHz (fast mode)
const CLK_ENABLE:  u16 = 1 << 8; // Bit 8 = clock output enable

// ---------------------------------------------------------------------------
// Module clock
// ---------------------------------------------------------------------------
const STBCR12: usize = 0xFCFE_0830;

// ---------------------------------------------------------------------------
// Per-port state
// ---------------------------------------------------------------------------

const NUM_PORTS: usize = 2;

struct SdhiState {
    waker: AtomicWaker,
    /// Accumulated INFO1 bits from ISR (ANDed with active mask at capture time).
    info1: AtomicU16,
    /// Accumulated INFO2 bits from ISR.
    info2: AtomicU16,
}

unsafe impl Sync for SdhiState {}

static STATE: [SdhiState; NUM_PORTS] = [
    SdhiState {
        waker: AtomicWaker::new(),
        info1: AtomicU16::new(0),
        info2: AtomicU16::new(0),
    },
    SdhiState {
        waker: AtomicWaker::new(),
        info1: AtomicU16::new(0),
        info2: AtomicU16::new(0),
    },
];

// ---------------------------------------------------------------------------
// GIC IRQ IDs
// ---------------------------------------------------------------------------

/// GIC IDs for SDHI0: status-change, operation, SDIO.
const SDHI0_IRQS: [u16; 3] = [302, 303, 304];
/// GIC IDs for SDHI1: status-change, operation, SDIO.
const SDHI1_IRQS: [u16; 3] = [305, 306, 307];

const SDHI_IRQ_PRIORITY: u8 = 10;

// ---------------------------------------------------------------------------
// Register helpers
// ---------------------------------------------------------------------------

#[inline]
fn port_base(port: u8) -> usize {
    match port {
        0 => SDHI0_BASE,
        _ => SDHI1_BASE,
    }
}

#[inline]
unsafe fn reg16(base: usize, off: usize) -> *mut u16 {
    (base + off) as *mut u16
}

#[inline]
unsafe fn reg32(base: usize, off: usize) -> *mut u32 {
    (base + off) as *mut u32
}

// ---------------------------------------------------------------------------
// Public API: hardware initialization
// ---------------------------------------------------------------------------

/// Initialize SDHI hardware for `port` (0 or 1).
///
/// Enables the module clock (STBCR12), releases soft-reset, configures
/// interrupt masks, and sets the SD timing option register.
///
/// Call once, with global IRQ disabled, before [`register_irqs`].
///
/// # Safety
/// Writes to memory-mapped peripheral registers.
pub unsafe fn init(port: u8) {
    // ---- enable SDHI module clock (STBCR12: bits 3:0 = SDHI11, 10, 01, 00) ----
    let stbcr12 = STBCR12 as *mut u8;
    stbcr12.write_volatile(0xF0); // enable both SDHI0 and SDHI1
    let _ = stbcr12.read_volatile(); // dummy read (required per RZ/A1 HW manual)

    let base = port_base(port);

    // ---- mask all meaningful interrupts (0 = enabled, 1 = masked) ----
    // Values match sd_init.c: SD_INFO1_MASK=0x031D, SD_INFO2_MASK=0x8B7F (RZA1)
    reg16(base, OFF_INFO1_MASK).write_volatile(0x031D);
    reg16(base, OFF_INFO2_MASK).write_volatile(0x8B7F);
    reg16(base, OFF_SDIO_INFO1_MASK).write_volatile(0xC007);

    // ---- clear SDIO mode ----
    reg16(base, OFF_SDIO_MODE).write_volatile(0x0000);

    // ---- clear status registers ----
    // Clear RESP(bit0) and DATA_TRNS(bit2) only, per sd_init.c: info1 & ~0x0005
    let info1 = reg16(base, OFF_INFO1).read_volatile();
    reg16(base, OFF_INFO1).write_volatile(info1 & !0x0005u16);
    reg16(base, OFF_INFO2).write_volatile(0x0000);
    reg16(base, OFF_SDIO_INFO1).write_volatile(0x0000);

    // ---- soft reset (RZ/A1 variant: 0x0006 → 0x0007) ----
    reg16(base, OFF_SOFT_RST).write_volatile(0x0006);
    reg16(base, OFF_SOFT_RST).write_volatile(0x0007);

    // ---- SD_OPTION: NCycle = SDCLK×2^23, 4-bit bus ----
    // Value 0x00BD: Rohan's tuning for faster card detect on the Deluge.
    reg16(base, OFF_OPTION).write_volatile(0x00BD);

    // ---- EXT_SWAP: no byte-swap ----
    reg16(base, OFF_EXT_SWAP).write_volatile(0x0000);

    // ---- CC_EXT_MODE: software transfer mode (no DMA yet) ----
    reg16(base, OFF_CC_EXT_MODE).write_volatile(0x0000);

    // ---- block size = 512 bytes ----
    reg16(base, OFF_SIZE).write_volatile(512);

    // ---- start at identification clock (~260 kHz) ----
    reg16(base, OFF_CLK_CTRL).write_volatile(CLK_DIV_256 | CLK_ENABLE);
}

/// Switch the SD clock to high-speed mode (~33 MHz).
///
/// Call after successful card initialization.
///
/// # Safety
/// Writes to memory-mapped SDHI register.
pub unsafe fn set_clock_fast(port: u8) {
    let base = port_base(port);
    // Wait for clock divider change to settle
    for _ in 0..10_000u32 {
        if reg16(base, OFF_INFO2).read_volatile() & INFO2_SCLKDIVEN != 0 {
            break;
        }
    }
    reg16(base, OFF_CLK_CTRL).write_volatile(CLK_DIV_2 | CLK_ENABLE);
    for _ in 0..10_000u32 {
        if reg16(base, OFF_INFO2).read_volatile() & INFO2_SCLKDIVEN != 0 {
            break;
        }
    }
}

/// Register GIC interrupt handlers for `port`.
///
/// Must be called while global IRQ is disabled.
///
/// # Safety
/// Writes to GIC registers.
pub unsafe fn register_irqs(port: u8) {
    let irqs = if port == 0 { SDHI0_IRQS } else { SDHI1_IRQS };
    let handlers: [gic::Handler; 3] = if port == 0 {
        [irq_sdhi0_cd, irq_sdhi0_op, irq_sdhi0_io]
    } else {
        [irq_sdhi1_cd, irq_sdhi1_op, irq_sdhi1_io]
    };
    for i in 0..3 {
        gic::register(irqs[i], handlers[i]);
        gic::set_priority(irqs[i], SDHI_IRQ_PRIORITY);
        gic::enable(irqs[i]);
    }
}

// ---------------------------------------------------------------------------
// Interrupt handlers (called by GIC dispatch)
// ---------------------------------------------------------------------------

fn irq_sdhi0_cd()  { unsafe { interrupt_handler(0); } }
fn irq_sdhi0_op()  { unsafe { interrupt_handler(0); } }
fn irq_sdhi0_io()  { /* SDIO not used; clear and ignore */ }

fn irq_sdhi1_cd()  { unsafe { interrupt_handler(1); } }
fn irq_sdhi1_op()  { unsafe { interrupt_handler(1); } }
fn irq_sdhi1_io()  { /* SDIO not used; clear and ignore */ }

/// Core interrupt handler: capture INFO1/INFO2, accumulate, wake waiter.
///
/// # Safety
/// Reads and writes memory-mapped SDHI registers.
pub unsafe fn interrupt_handler(port: u8) {
    let base = port_base(port);

    // Read status
    let info1 = reg16(base, OFF_INFO1).read_volatile();
    let info2 = reg16(base, OFF_INFO2).read_volatile();

    // Clear bits by writing 0 (write-0-to-clear per RZ/A1 HW manual).
    // Only clear the bits we just read to avoid losing a racing edge.
    reg16(base, OFF_INFO1).write_volatile(!info1);
    reg16(base, OFF_INFO2).write_volatile(!info2);

    // Also clear SDIO status to prevent spurious SDIO interrupts.
    let sdio = reg16(base, OFF_SDIO_INFO1).read_volatile();
    reg16(base, OFF_SDIO_INFO1).write_volatile(!sdio);

    let st = &STATE[port as usize];
    st.info1.fetch_or(info1, Ordering::Release);
    st.info2.fetch_or(info2, Ordering::Release);
    st.waker.wake();
}

// ---------------------------------------------------------------------------
// Async waiting primitives
// ---------------------------------------------------------------------------

/// Take (atomically consume) accumulated INFO1/INFO2 bits.
fn take_info(port: u8) -> (u16, u16) {
    let st = &STATE[port as usize];
    let i1 = st.info1.swap(0, Ordering::Acquire);
    let i2 = st.info2.swap(0, Ordering::Acquire);
    (i1, i2)
}

/// Peek at INFO1/INFO2 without consuming.
fn peek_info(port: u8) -> (u16, u16) {
    let st = &STATE[port as usize];
    let i1 = st.info1.load(Ordering::Acquire);
    let i2 = st.info2.load(Ordering::Acquire);
    (i1, i2)
}

/// Clear accumulated state.
fn clear_info(port: u8) {
    let st = &STATE[port as usize];
    st.info1.store(0, Ordering::Release);
    st.info2.store(0, Ordering::Release);
}

// ---------------------------------------------------------------------------
// Command/argument helpers
// ---------------------------------------------------------------------------

/// Set the 32-bit command argument (split into two 16-bit registers).
///
/// # Safety
/// Writes to SDHI registers.
#[inline]
pub unsafe fn set_arg(port: u8, arg: u32) {
    let base = port_base(port);
    reg16(base, OFF_ARG0).write_volatile((arg & 0xFFFF) as u16);
    reg16(base, OFF_ARG1).write_volatile((arg >> 16) as u16);
}

/// Wait for the clock divider register to be ready (SCLKDIVEN bit).
///
/// # Safety
/// Reads SDHI INFO2 register.
pub unsafe fn wait_clk_stable(port: u8) {
    let base = port_base(port);
    for _ in 0..100_000u32 {
        if reg16(base, OFF_INFO2).read_volatile() & INFO2_SCLKDIVEN != 0 {
            return;
        }
    }
    // Timeout — continue anyway; hardware may still work.
}

// ---------------------------------------------------------------------------
// Async command issue
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SdhiError {
    /// Response timeout (no response received in time).
    ResponseTimeout,
    /// Response CRC error.
    ResponseCrc,
    /// Data CRC error.
    DataCrc,
    /// Data timeout.
    DataTimeout,
    /// SDHI command busy timeout.
    CmdBusy,
    /// Illegal register access.
    IllegalAccess,
    /// Generic hardware error.
    HardwareError,
}

impl From<SdhiError> for &'static str {
    fn from(e: SdhiError) -> &'static str {
        match e {
            SdhiError::ResponseTimeout => "SDHI: response timeout",
            SdhiError::ResponseCrc     => "SDHI: response CRC error",
            SdhiError::DataCrc         => "SDHI: data CRC error",
            SdhiError::DataTimeout     => "SDHI: data timeout",
            SdhiError::CmdBusy         => "SDHI: command busy timeout",
            SdhiError::IllegalAccess   => "SDHI: illegal register access",
            SdhiError::HardwareError   => "SDHI: hardware error",
        }
    }
}

fn check_info2_errors(info2: u16) -> Result<(), SdhiError> {
    if info2 & INFO2_ILA   != 0 { return Err(SdhiError::IllegalAccess); }
    if info2 & INFO2_ERR5  != 0 { return Err(SdhiError::ResponseTimeout); }
    if info2 & INFO2_ERR6  != 0 { return Err(SdhiError::ResponseTimeout); }
    if info2 & INFO2_ERR0  != 0 { return Err(SdhiError::ResponseCrc); }
    if info2 & INFO2_ERR1  != 0 { return Err(SdhiError::ResponseCrc); }
    if info2 & INFO2_ERR2  != 0 { return Err(SdhiError::DataCrc); }
    if info2 & INFO2_ERR3  != 0 { return Err(SdhiError::DataTimeout); }
    if info2 & INFO2_ERR4  != 0 { return Err(SdhiError::DataCrc); }
    Ok(())
}

/// Issue an SD command and wait asynchronously for the response end.
///
/// `cmd_val` is written directly to the SD_CMD register.
/// The argument should have been set via [`set_arg`] immediately before.
///
/// Returns `Ok(())` when INFO1_RESP is set; `Err` on hardware error.
///
/// # Safety
/// Reads/writes SDHI peripheral registers. Must not be called concurrently
/// for the same port.
pub async unsafe fn send_cmd(port: u8, cmd_val: u16) -> Result<(), SdhiError> {
    let base = port_base(port);

    // Wait for clock to be stable
    wait_clk_stable(port);

    // Clear any stale interrupt state
    clear_info(port);

    // Enable response-end and error interrupts.
    // Hardware polarity: 0 = enabled, 1 = masked → write bitwise complement.
    // INFO1: enable RESP only  → ~0x0001 = 0xFFFE
    // INFO2: enable ERR_ALL (includes ILA) → ~INFO2_ERR_ALL = 0x7F80
    reg16(base, OFF_INFO1_MASK).write_volatile(!INFO1_RESP);
    reg16(base, OFF_INFO2_MASK).write_volatile(!INFO2_ERR_ALL);

    // Issue command
    reg16(base, OFF_CMD).write_volatile(cmd_val);

    // Async wait
    let result = poll_fn(|cx| {
        let st = &STATE[port as usize];
        let i1 = st.info1.load(Ordering::Acquire);
        let i2 = st.info2.load(Ordering::Acquire);

        // Check errors first
        if let Err(e) = check_info2_errors(i2) {
            return Poll::Ready(Err(e));
        }
        // Check success
        if i1 & INFO1_RESP != 0 {
            return Poll::Ready(Ok(()));
        }
        // Still waiting — register waker
        st.waker.register(cx.waker());
        // Re-check in case interrupt fired between the load and register
        let i1 = st.info1.load(Ordering::Acquire);
        let i2 = st.info2.load(Ordering::Acquire);
        if let Err(e) = check_info2_errors(i2) {
            return Poll::Ready(Err(e));
        }
        if i1 & INFO1_RESP != 0 {
            return Poll::Ready(Ok(()));
        }
        Poll::Pending
    })
    .await;

    // Disable all interrupts (0 = enabled, 1 = masked → 0xFFFF masks all)
    reg16(base, OFF_INFO1_MASK).write_volatile(0xFFFF);
    reg16(base, OFF_INFO2_MASK).write_volatile(0xFFFF);

    // Clear accumulated state
    clear_info(port);

    result
}

// ---------------------------------------------------------------------------
// Response reading
// ---------------------------------------------------------------------------

/// Read the 32-bit R1/R3/R6/R7 response (RESP0 + RESP1 combined).
///
/// # Safety
/// Reads SDHI response registers.
pub unsafe fn read_r1(port: u8) -> u32 {
    let base = port_base(port);
    let lo = reg16(base, OFF_RESP0).read_volatile() as u32;
    let hi = reg16(base, OFF_RESP1).read_volatile() as u32;
    (hi << 16) | lo
}

/// Read the full 128-bit R2 response (CID or CSD register).
///
/// Returns `[word0, word1, word2, word3]` where word0 contains bits [127:96].
///
/// # Safety
/// Reads SDHI response registers.
pub unsafe fn read_r2(port: u8) -> [u32; 4] {
    let base = port_base(port);
    let r = |off: usize| reg16(base, off).read_volatile() as u32;
    [
        (r(OFF_RESP7) << 16) | r(OFF_RESP6),
        (r(OFF_RESP5) << 16) | r(OFF_RESP4),
        (r(OFF_RESP3) << 16) | r(OFF_RESP2),
        (r(OFF_RESP1) << 16) | r(OFF_RESP0),
    ]
}

// ---------------------------------------------------------------------------
// Software (FIFO) data transfer
// ---------------------------------------------------------------------------

/// Read `count` 512-byte blocks from the card into `buf`.
///
/// Caller must have already:
///  1. Set block count: [`set_block_count`]
///  2. Issued CMD17 or CMD18 via [`send_cmd`]
///
/// Transfer is interrupt-driven: the hardware raises INFO2_BRE when the FIFO
/// has 512 bytes ready; we drain it, then repeat for each block.  After all
/// blocks arrive, we wait for INFO1_DATA_TRNS (access end).
///
/// # Safety
/// Reads/writes SDHI registers and the provided buffer.
pub async unsafe fn read_blocks_sw(
    port: u8,
    buf: *mut u8,
    count: u32,
) -> Result<(), SdhiError> {
    let base = port_base(port);

    // Enable DATA_TRNS (access end), BRE, and all errors.
    // Hardware polarity: 0 = enabled, 1 = masked → write bitwise complement.
    // INFO1: enable DATA_TRNS only  → ~INFO1_DATA_TRNS = 0xFFFB
    // INFO2: enable ERR_ALL + BRE   → ~(INFO2_ERR_ALL | INFO2_BRE) = 0x7E80
    reg16(base, OFF_INFO1_MASK).write_volatile(!INFO1_DATA_TRNS);
    reg16(base, OFF_INFO2_MASK).write_volatile(!(INFO2_ERR_ALL | INFO2_BRE));

    let mut dst = buf;
    for _block in 0..count {
        // Wait for Buffer Read Enable
        poll_fn(|cx| {
            let st = &STATE[port as usize];
            let i2 = st.info2.load(Ordering::Acquire);
            if i2 & INFO2_BRE != 0 {
                // Consume the BRE bit only
                st.info2.fetch_and(!INFO2_BRE, Ordering::AcqRel);
                return Poll::Ready(Ok::<(), SdhiError>(()));
            }
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            st.waker.register(cx.waker());
            // Double-check
            let i2 = st.info2.load(Ordering::Acquire);
            if i2 & INFO2_BRE != 0 {
                st.info2.fetch_and(!INFO2_BRE, Ordering::AcqRel);
                return Poll::Ready(Ok(()));
            }
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            Poll::Pending
        })
        .await?;

        // Drain 512 bytes from 32-bit FIFO (128 reads)
        let fifo = reg32(base, OFF_BUF0);
        for w in 0..128usize {
            let word = fifo.read_volatile();
            let p = dst.add(w * 4) as *mut u32;
            p.write_unaligned(word);
        }
        dst = dst.add(512);
    }

    // Wait for access end (DATA_TRNS)
    poll_fn(|cx| {
        let st = &STATE[port as usize];
        let i1 = st.info1.load(Ordering::Acquire);
        let i2 = st.info2.load(Ordering::Acquire);
        if let Err(e) = check_info2_errors(i2) {
            return Poll::Ready(Err(e));
        }
        if i1 & INFO1_DATA_TRNS != 0 {
            return Poll::Ready(Ok(()));
        }
        st.waker.register(cx.waker());
        let i1 = st.info1.load(Ordering::Acquire);
        let i2 = st.info2.load(Ordering::Acquire);
        if let Err(e) = check_info2_errors(i2) {
            return Poll::Ready(Err(e));
        }
        if i1 & INFO1_DATA_TRNS != 0 {
            return Poll::Ready(Ok(()));
        }
        Poll::Pending
    })
    .await?;

    // Disable all interrupts (0xFFFF = all masked)
    reg16(base, OFF_INFO1_MASK).write_volatile(0xFFFF);
    reg16(base, OFF_INFO2_MASK).write_volatile(0xFFFF);
    clear_info(port);

    Ok(())
}

/// Write `count` 512-byte blocks from `buf` to the card.
///
/// Caller must have already set block count and issued CMD24 or CMD25.
///
/// # Safety
/// Reads/writes SDHI registers and the provided buffer.
pub async unsafe fn write_blocks_sw(
    port: u8,
    buf: *const u8,
    count: u32,
) -> Result<(), SdhiError> {
    let base = port_base(port);

    // Enable DATA_TRNS, BWE, and all errors.
    // Hardware polarity: 0 = enabled, 1 = masked → write bitwise complement.
    // INFO1: enable DATA_TRNS only  → ~INFO1_DATA_TRNS = 0xFFFB
    // INFO2: enable ERR_ALL + BWE   → ~(INFO2_ERR_ALL | INFO2_BWE) = 0x7D80
    reg16(base, OFF_INFO1_MASK).write_volatile(!INFO1_DATA_TRNS);
    reg16(base, OFF_INFO2_MASK).write_volatile(!(INFO2_ERR_ALL | INFO2_BWE));

    let mut src = buf;
    for _block in 0..count {
        // Wait for Buffer Write Enable
        poll_fn(|cx| {
            let st = &STATE[port as usize];
            let i2 = st.info2.load(Ordering::Acquire);
            if i2 & INFO2_BWE != 0 {
                st.info2.fetch_and(!INFO2_BWE, Ordering::AcqRel);
                return Poll::Ready(Ok::<(), SdhiError>(()));
            }
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            st.waker.register(cx.waker());
            let i2 = st.info2.load(Ordering::Acquire);
            if i2 & INFO2_BWE != 0 {
                st.info2.fetch_and(!INFO2_BWE, Ordering::AcqRel);
                return Poll::Ready(Ok(()));
            }
            if let Err(e) = check_info2_errors(i2) {
                return Poll::Ready(Err(e));
            }
            Poll::Pending
        })
        .await?;

        // Fill 512 bytes into 32-bit FIFO (128 writes)
        let fifo = reg32(base, OFF_BUF0);
        for w in 0..128usize {
            let p = src.add(w * 4) as *const u32;
            fifo.write_volatile(p.read_unaligned());
        }
        src = src.add(512);
    }

    // Wait for access end
    poll_fn(|cx| {
        let st = &STATE[port as usize];
        let i1 = st.info1.load(Ordering::Acquire);
        let i2 = st.info2.load(Ordering::Acquire);
        if let Err(e) = check_info2_errors(i2) {
            return Poll::Ready(Err(e));
        }
        if i1 & INFO1_DATA_TRNS != 0 {
            return Poll::Ready(Ok(()));
        }
        st.waker.register(cx.waker());
        let i1 = st.info1.load(Ordering::Acquire);
        let i2 = st.info2.load(Ordering::Acquire);
        if let Err(e) = check_info2_errors(i2) {
            return Poll::Ready(Err(e));
        }
        if i1 & INFO1_DATA_TRNS != 0 {
            return Poll::Ready(Ok(()));
        }
        Poll::Pending
    })
    .await?;

    reg16(base, OFF_INFO1_MASK).write_volatile(0xFFFF);
    reg16(base, OFF_INFO2_MASK).write_volatile(0xFFFF);
    clear_info(port);

    Ok(())
}

// ---------------------------------------------------------------------------
// Block-count / stop-transfer helpers
// ---------------------------------------------------------------------------

/// Set the number of blocks for a multi-block transfer.
///
/// Also writes SD_STOP to auto-stop after the given count (CMD12 auto-issue).
///
/// # Safety
/// Writes SDHI registers.
pub unsafe fn set_block_count(port: u8, count: u32) {
    let base = port_base(port);
    reg16(base, OFF_SECCNT).write_volatile(count as u16);
    // Auto-stop enable (bit 8) when count > 1; otherwise no auto-stop.
    if count > 1 {
        reg16(base, OFF_STOP).write_volatile(0x0100); // SEC bit: auto-issue CMD12
    } else {
        reg16(base, OFF_STOP).write_volatile(0x0000);
    }
}

/// Issue CMD12 (STOP_TRANSMISSION) to abort a multi-block transfer.
///
/// # Safety
/// Writes SDHI registers.
pub async unsafe fn stop_transfer(port: u8) -> Result<(), SdhiError> {
    set_arg(port, 0);
    send_cmd(port, 12u16).await // CMD12
}

// ---------------------------------------------------------------------------
// Card detect
// ---------------------------------------------------------------------------

/// Return `true` if a card is physically inserted (CD pin low = card present).
///
/// Reads INFO1 directly from hardware (not the accumulated atomic).
///
/// # Safety
/// Reads SDHI INFO1 register.
pub unsafe fn card_inserted(port: u8) -> bool {
    let base = port_base(port);
    // INFO1 bit 5 (INS_CD) is latched high when card is present.
    // On RZ/A1L, CD pin pulled low when card inserted.
    // Direct register read vs. accumulated state.
    let info1 = reg16(base, OFF_INFO1).read_volatile();
    // Use DAT3 detect: bit 9 (INS_DAT3) or CD pin bit 4 (INS_CD).
    // The Deluge uses socket CD, so check bit 4.
    info1 & INFO1_INS_CD != 0
}

/// Enable card-detect interrupts (insert + remove) in INFO1_MASK.
///
/// # Safety
/// Writes SDHI INFO1_MASK register.
pub unsafe fn enable_card_detect_irq(port: u8) {
    let base = port_base(port);
    // Hardware polarity: 0 = enabled, 1 = masked → clear bits to enable.
    let mask = reg16(base, OFF_INFO1_MASK).read_volatile();
    reg16(base, OFF_INFO1_MASK).write_volatile(mask & !INFO1_DET_CD);
}

// ---------------------------------------------------------------------------
// Card capacity (set by deluge-bsp after CSD decode)
// ---------------------------------------------------------------------------

/// Total card capacity in 512-byte blocks, per port.
///
/// Populated by the BSP layer via [`set_card_blocks`] during card init so
/// that the polling `BlockDevice` impl can return a reliable block count.
static CARD_BLOCKS: [AtomicU32; NUM_PORTS] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
];

/// Store the card capacity (in 512-byte blocks) for `port`.
///
/// Called by the BSP once the CSD register has been decoded.
pub fn set_card_blocks(port: u8, blocks: u32) {
    CARD_BLOCKS[port as usize].store(blocks, Ordering::Release);
}

/// Return the stored card capacity (in 512-byte blocks) for `port`.
///
/// Returns 0 if [`set_card_blocks`] has not been called yet.
pub fn card_size_blocks(port: u8) -> u32 {
    CARD_BLOCKS[port as usize].load(Ordering::Acquire)
}

// ---------------------------------------------------------------------------
// Polling (synchronous, executor-free) data transfer
// ---------------------------------------------------------------------------
//
// These functions mirror the async read_blocks_sw / write_blocks_sw paths
// but spin on hardware register bits instead of using AtomicWaker.  They are
// safe to call outside of an Embassy executor (e.g. from embedded-sdmmc's
// synchronous BlockDevice trait impl).
//
// A timeout counter guards against hardware hang.  The limit is chosen to be
// comfortably above the maximum expected transfer time for 512-byte blocks at
// the fast SD clock (~33 MHz):  512 bytes / 33 MHz ≈ 120 µs → ≈ 4 000 ticks
// at P0 / 8.  We use 10 000 000 to allow for card back-off without being
// infinite.

const POLL_TIMEOUT: u32 = 10_000_000;

/// Issue an SD command using register polling (no IRQ / no executor).
///
/// Equivalent to [`send_cmd`] but spins on INFO1_RESP / INFO2_ERR_ALL.
///
/// # Safety
/// Must not be called concurrently on the same port.  Writes SDHI registers.
pub unsafe fn send_cmd_poll(port: u8, cmd_val: u16) -> Result<(), SdhiError> {
    let base = port_base(port);

    wait_clk_stable(port);
    clear_info(port);

    // Mask all interrupts (we poll instead).
    reg16(base, OFF_INFO1_MASK).write_volatile(0xFFFF);
    reg16(base, OFF_INFO2_MASK).write_volatile(0xFFFF);

    // Issue command.
    reg16(base, OFF_CMD).write_volatile(cmd_val);

    // Spin until RESP or error.
    for _ in 0..POLL_TIMEOUT {
        let info1 = reg16(base, OFF_INFO1).read_volatile();
        let info2 = reg16(base, OFF_INFO2).read_volatile();

        if info2 & INFO2_ERR_ALL != 0 {
            // Clear before returning.
            reg16(base, OFF_INFO1).write_volatile(!info1);
            reg16(base, OFF_INFO2).write_volatile(!info2);
            clear_info(port);
            return check_info2_errors(info2);
        }
        if info1 & INFO1_RESP != 0 {
            reg16(base, OFF_INFO1).write_volatile(!info1);
            reg16(base, OFF_INFO2).write_volatile(!info2);
            clear_info(port);
            return Ok(());
        }
    }

    clear_info(port);
    Err(SdhiError::ResponseTimeout)
}

/// Read `count` 512-byte blocks from the card using register polling.
///
/// The caller must have already:
///   1. Set block count via [`set_block_count`].
///   2. Issued the read command via [`send_cmd_poll`].
///
/// # Safety
/// Must not be called concurrently on the same port.
pub unsafe fn read_blocks_poll(
    port: u8,
    buf: *mut u8,
    count: u32,
) -> Result<(), SdhiError> {
    let base = port_base(port);

    let mut dst = buf;
    for _block in 0..count {
        // Wait for Buffer Read Enable (INFO2_BRE).
        let mut ready = false;
        for _ in 0..POLL_TIMEOUT {
            let info2 = reg16(base, OFF_INFO2).read_volatile();
            if info2 & INFO2_ERR_ALL != 0 {
                reg16(base, OFF_INFO2).write_volatile(!info2);
                return check_info2_errors(info2);
            }
            if info2 & INFO2_BRE != 0 {
                // Clear only BRE.
                reg16(base, OFF_INFO2).write_volatile(!(INFO2_BRE));
                ready = true;
                break;
            }
        }
        if !ready {
            return Err(SdhiError::DataTimeout);
        }

        // Drain 512 bytes from 32-bit FIFO (128 × 32-bit reads).
        let fifo = reg32(base, OFF_BUF0);
        for w in 0..128usize {
            let word = fifo.read_volatile();
            let p = dst.add(w * 4) as *mut u32;
            p.write_unaligned(word);
        }
        dst = dst.add(512);
    }

    // Wait for access end (INFO1_DATA_TRNS).
    for _ in 0..POLL_TIMEOUT {
        let info2 = reg16(base, OFF_INFO2).read_volatile();
        if info2 & INFO2_ERR_ALL != 0 {
            reg16(base, OFF_INFO2).write_volatile(!info2);
            return check_info2_errors(info2);
        }
        let info1 = reg16(base, OFF_INFO1).read_volatile();
        if info1 & INFO1_DATA_TRNS != 0 {
            reg16(base, OFF_INFO1).write_volatile(!info1);
            reg16(base, OFF_INFO2).write_volatile(!info2);
            return Ok(());
        }
    }

    Err(SdhiError::DataTimeout)
}

/// Write `count` 512-byte blocks to the card using register polling.
///
/// The caller must have already set block count and issued the write command.
///
/// # Safety
/// Must not be called concurrently on the same port.
pub unsafe fn write_blocks_poll(
    port: u8,
    buf: *const u8,
    count: u32,
) -> Result<(), SdhiError> {
    let base = port_base(port);

    let mut src = buf;
    for _block in 0..count {
        // Wait for Buffer Write Enable (INFO2_BWE).
        let mut ready = false;
        for _ in 0..POLL_TIMEOUT {
            let info2 = reg16(base, OFF_INFO2).read_volatile();
            if info2 & INFO2_ERR_ALL != 0 {
                reg16(base, OFF_INFO2).write_volatile(!info2);
                return check_info2_errors(info2);
            }
            if info2 & INFO2_BWE != 0 {
                reg16(base, OFF_INFO2).write_volatile(!(INFO2_BWE));
                ready = true;
                break;
            }
        }
        if !ready {
            return Err(SdhiError::DataTimeout);
        }

        // Fill 512 bytes into 32-bit FIFO (128 × 32-bit writes).
        let fifo = reg32(base, OFF_BUF0);
        for w in 0..128usize {
            let p = src.add(w * 4) as *const u32;
            fifo.write_volatile(p.read_unaligned());
        }
        src = src.add(512);
    }

    // Wait for access end.
    for _ in 0..POLL_TIMEOUT {
        let info2 = reg16(base, OFF_INFO2).read_volatile();
        if info2 & INFO2_ERR_ALL != 0 {
            reg16(base, OFF_INFO2).write_volatile(!info2);
            return check_info2_errors(info2);
        }
        let info1 = reg16(base, OFF_INFO1).read_volatile();
        if info1 & INFO1_DATA_TRNS != 0 {
            reg16(base, OFF_INFO1).write_volatile(!info1);
            reg16(base, OFF_INFO2).write_volatile(!info2);
            return Ok(());
        }
    }

    Err(SdhiError::DataTimeout)
}

// ---------------------------------------------------------------------------
// Owned typed wrapper
// ---------------------------------------------------------------------------

/// Owned handle for SDHI port `PORT` (0 or 1).
///
/// Wraps the free functions in an object-oriented API.  The type parameter
/// `PORT` encodes the port number at compile time.
pub struct Sdhi<const PORT: u8>;

impl<const PORT: u8> Sdhi<PORT> {
    /// Claim ownership of SDHI port `PORT`.
    ///
    /// # Safety
    /// The caller must ensure no other code uses port `PORT` concurrently, and
    /// that the SDHI clock has been enabled (via `stb::init` or equivalent).
    pub unsafe fn new() -> Self { Sdhi }

    /// Initialise the hardware for this port.
    ///
    /// # Safety
    /// Writes memory-mapped SDHI registers.
    pub unsafe fn init(&self) { init(PORT) }

    /// Switch the SD clock to high speed (12.5 MHz with MPB3 = 25 MHz).
    ///
    /// # Safety
    /// Writes memory-mapped SDHI registers.
    pub unsafe fn set_clock_fast(&self) { set_clock_fast(PORT) }

    /// Register GIC IRQs for this port's interrupt handler.
    ///
    /// # Safety
    /// Writes GIC registers.
    pub unsafe fn register_irqs(&self) { register_irqs(PORT) }

    /// Returns `true` if a card is detected in the slot.
    ///
    /// # Safety
    /// Reads memory-mapped SDHI registers.
    pub unsafe fn card_inserted(&self) -> bool { card_inserted(PORT) }

    /// Set the number of blocks for a multi-block transfer command.
    ///
    /// # Safety
    /// Writes memory-mapped SDHI registers.
    pub unsafe fn set_block_count(&self, count: u32) { set_block_count(PORT, count) }

    /// Issue an SD command and wait (polling) for a response.
    ///
    /// # Safety
    /// Must not be called concurrently.  Writes SDHI registers.
    pub unsafe fn send_cmd_poll(&self, cmd_val: u16) -> Result<(), SdhiError> {
        send_cmd_poll(PORT, cmd_val)
    }

    /// Read `count` 512-byte blocks into `buf` using register polling.
    ///
    /// # Safety
    /// Must not be called concurrently.  `buf` must be valid for `count * 512` bytes.
    pub unsafe fn read_blocks_poll(&self, buf: *mut u8, count: u32) -> Result<(), SdhiError> {
        read_blocks_poll(PORT, buf, count)
    }

    /// Write `count` 512-byte blocks from `buf` using register polling.
    ///
    /// # Safety
    /// Must not be called concurrently.  `buf` must be valid for `count * 512` bytes.
    pub unsafe fn write_blocks_poll(&self, buf: *const u8, count: u32) -> Result<(), SdhiError> {
        write_blocks_poll(PORT, buf, count)
    }
}

// ---------------------------------------------------------------------------
// Unit tests (host-side, register address verification only)
// ---------------------------------------------------------------------------

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn sdhi1_base_address() {
        assert_eq!(SDHI1_BASE, 0xE804_E800);
    }

    #[test]
    fn register_offsets() {
        // Verify register layout: offset = register_index * 2
        assert_eq!(OFF_CMD,        0x00);
        assert_eq!(OFF_ARG0,       0x08);
        assert_eq!(OFF_ARG1,       0x0C);
        assert_eq!(OFF_SECCNT,     0x14);
        assert_eq!(OFF_INFO1,      0x38);
        assert_eq!(OFF_INFO2,      0x3C);
        assert_eq!(OFF_INFO1_MASK, 0x40);
        assert_eq!(OFF_INFO2_MASK, 0x44);
        assert_eq!(OFF_CLK_CTRL,   0x48);
        assert_eq!(OFF_SIZE,       0x4C);
        assert_eq!(OFF_OPTION,     0x50);
        assert_eq!(OFF_BUF0,       0x60);
        assert_eq!(OFF_CC_EXT_MODE, 0x1B0);
        assert_eq!(OFF_SOFT_RST,   0x1C0);
        assert_eq!(OFF_EXT_SWAP,   0x1E0);
    }

    #[test]
    fn info_bit_masks() {
        assert_eq!(INFO1_RESP,      0x0001);
        assert_eq!(INFO1_DATA_TRNS, 0x0004);
        assert_eq!(INFO2_BRE,       0x0100);
        assert_eq!(INFO2_BWE,       0x0200);
        assert_eq!(INFO2_CBSY,      0x4000);
    }

    #[test]
    fn gic_ids_for_sdhi1() {
        assert_eq!(SDHI1_IRQS, [305, 306, 307]);
    }
}
