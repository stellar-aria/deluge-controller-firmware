//! Synchronous Serial Interface (SSIF) driver for the RZ/A1L — 44.1 kHz I²S.
//!
//! Implements stereo audio streaming on SSI channel 0 using DMA circular
//! link-descriptor mode.  Only SSI0 is wired to the audio codec on Deluge
//! hardware; the other five channels are unused.
//!
//! ## Clock derivation
//! ```text
//! AUDIO_X1 ≈ 22.5792 MHz  (external crystal on the Deluge board)
//! ÷ 4  (CKDV = 0b0010)  →  BCLK = 5.6448 MHz
//! ÷ 64 (2 ch × 32-bit SWL)  →  88 200 half-frames / s
//! ÷ 2  →  44 100 Hz sample rate
//! ```
//!
//! ## DMA layout
//! | Channel | Direction       | GIC IRQ |
//! |---------|-----------------|---------|
//! | DMA 6   | SRAM → SSIFTDR  | 47      |
//! | DMA 7   | SSIFRDR → SRAM  | 48      |
//!
//! Both DMA channels run in self-referential circular link-descriptor mode:
//! the `next` pointer in each descriptor points back to itself, so the DMA
//! engine re-arms automatically after each pass through the buffer.
//!
//! ## Buffer sizes
//! | Buffer | Frames  | Stereo samples | Bytes  | Duration |
//! |--------|---------|----------------|--------|----------|
//! | TX     | 1 024   | 2 048          | 8 192  | ~23.2 ms |
//! | RX     | 2 048   | 4 096          | 16 384 | ~46.4 ms |
//!
//! ## Uncached access
//! DMA transfers bypass the CPU cache.  Any CPU code that reads/writes the
//! sample buffers must use the **uncached mirror** alias
//! (`physical address + UNCACHED_MIRROR_OFFSET`) to avoid stale cache lines.
//! [`tx_buf_start`], [`tx_buf_end`], [`rx_buf_start`], [`rx_buf_end`],
//! [`tx_current_ptr`], and [`rx_current_ptr`] all return uncached pointers.

use crate::dmac;

// ── Channel / resource constants ────────────────────────────────────────────

/// SSI hardware channel used for audio (SSI0).
pub const SSI_CH: u8 = 0;
/// DMA channel for SSI0 TX (SRAM → SSIFTDR).
pub const TX_DMA_CH: u8 = 6;
/// DMA channel for SSI0 RX (SSIFRDR → SRAM).
pub const RX_DMA_CH: u8 = 7;

/// DMARS resource-selector value for SSI0 transmit.
const DMARS_SSI0_TX: u32 = 0x00E1;
/// DMARS resource-selector value for SSI0 receive.
const DMARS_SSI0_RX: u32 = 0x00E2;

/// Level-triggered DMA flag bit used in CHCFG (bit 6).
const DMA_LVL: u32 = 1 << 6;

// ── SSI0 register map ────────────────────────────────────────────────────────

const SSI0_BASE:    usize = 0xE820_B000;
const SSICR_OFF:    usize = 0x00; // Control register
const SSIFCR_OFF:   usize = 0x10; // FIFO control register
const SSIFTDR_OFF:  usize = 0x18; // TX FIFO data register
const SSIFRDR_OFF:  usize = 0x1C; // RX FIFO data register
const SSITDMR_OFF:  usize = 0x20; // TDM mode register

/// SSICR initialisation value (copied verbatim from `drv_ssif_user.h`):
///   - CKS=0 : AUDIO_X1 clock source
///   - CHNL=00: 1 channel pair (stereo)
///   - DWL=101: 24-bit data word
///   - SWL=011: 32-bit system word
///   - SCKD=1 : RZ/A1L generates BCLK (master)
///   - SWSD=1 : RZ/A1L generates LRCK (master)
///   - SCKP=SWSP=SPDP=SDTA=PDTA=DEL=0 (all defaults)
///   - CKDV=0010: AUDIO_X1 ÷ 4
const SSICR_INIT:  u32 = 0x003B_C020;

/// SSIFCR initialisation value:
///   - TFRST=1, RFRST=1: both FIFOs held in reset until `ssiStart`
///   - TTRG=101: TX-empty trigger level
///   - TIE=0, RIE=0: interrupts disabled until FIFOs are released
const SSIFCR_INIT: u32 = 0x0000_00A3;

// ── CPG soft-reset ───────────────────────────────────────────────────────────

/// CPG SWRSTCR1 — byte register that controls peripheral soft-reset.
/// Bit `(6 - ssi_channel)` asserts / deasserts the SSI reset.
const SWRSTCR1: usize = 0xFCFE_0460;

// ── Uncached mirror ──────────────────────────────────────────────────────────

/// Add this offset to any cached internal-SRAM virtual address to obtain its
/// uncached alias.  DMA transfers go directly to the physical bus, bypassing
/// the CPU cache, so the CPU must access DMA buffers through this alias.
pub const UNCACHED_MIRROR_OFFSET: usize = 0x4000_0000;

// ── TX / RX sample buffers ───────────────────────────────────────────────────

/// TX buffer depth: stereo frames.
pub const TX_FRAMES: usize = 1024;
/// RX buffer depth: stereo frames.
pub const RX_FRAMES: usize = 2048;

/// Total TX buffer length in `i32` samples (TX_FRAMES × 2 mono channels).
pub const TX_BUF_LEN: usize = TX_FRAMES * 2;
/// Total RX buffer length in `i32` samples (RX_FRAMES × 2 mono channels).
pub const RX_BUF_LEN: usize = RX_FRAMES * 2;

/// Cache-line-aligned wrapper so `repr(align(32))` can apply to a fixed-size
/// array.  The DMA hardware requires descriptors and buffers to start on a
/// 32-byte (cache-line) boundary.
#[repr(align(32))]
struct Aligned32<const N: usize>([i32; N]);

// SAFETY: these buffers are only accessed through the uncached alias during
// normal operation; the DMA engine accesses them on the bus side.
static mut TX_BUF: Aligned32<TX_BUF_LEN> = Aligned32([0i32; TX_BUF_LEN]);
static mut RX_BUF: Aligned32<RX_BUF_LEN> = Aligned32([0i32; RX_BUF_LEN]);

// ── DMA link descriptors ─────────────────────────────────────────────────────

/// An 8-word, cache-line-aligned DMA link descriptor.
///
/// Word layout (matching the Renesas RZ/A1L DMA hardware):
/// | Word | Field       | Description                                |
/// |------|-------------|--------------------------------------------|
/// | 0    | HEADER      | Link-descriptor control flags (0b1101)     |
/// | 1    | N0SA / src  | Source address                             |
/// | 2    | N0DA / dst  | Destination address                        |
/// | 3    | N0TB / size | Transfer byte count                        |
/// | 4    | CHCFG       | Channel config (loaded into CHCFG register)|
/// | 5    | CHITVL      | Interval register (0 = no delay)           |
/// | 6    | CHEXT       | Extension register (0)                     |
/// | 7    | NXLA        | Address of the *next* descriptor (patched  |
/// |      |             | at runtime to point back to self)          |
#[repr(C, align(32))]
struct LinkDesc([u32; 8]);

// CHCFG values for the two SSI DMA channels, computed at compile time from
// the same formula as the C driver:
//
//   TX: 0b10000001_00100010_00100010_00101000 | DMA_LVL | (TX_DMA_CH & 7)
//       = 0x8122_2228 | 0x40 | 0x06  = 0x8122_226E
//   RX: 0b10000001_00010010_00100010_00100000 | DMA_LVL | (RX_DMA_CH & 7)
//       = 0x8112_2220 | 0x40 | 0x07  = 0x8112_2267
//
// The TX config increments the source address (memory side) and keeps the
// destination fixed (SSIFTDR hardware register).  The RX config does the
// reverse.

const TX_CHCFG: u32 = 0x8122_2228 | DMA_LVL | (TX_DMA_CH as u32 & 7);
const RX_CHCFG: u32 = 0x8112_2220 | DMA_LVL | (RX_DMA_CH as u32 & 7);

/// TX link descriptor.  The `src` (word 1) and `next` (word 7) fields are
/// patched at runtime by [`init`] once the static addresses are known.
static mut TX_DESC: LinkDesc = LinkDesc([
    0b1101,                                    // Header: LDEN + NXA + valid
    0,                                         // src  (→ TX_BUF, set at init)
    (SSI0_BASE + SSIFTDR_OFF) as u32,          // dst  = SSIFTDR (fixed)
    (TX_BUF_LEN * core::mem::size_of::<i32>()) as u32, // transfer bytes
    TX_CHCFG,                                  // CHCFG
    0,                                         // CHITVL (no interval)
    0,                                         // CHEXT
    0,                                         // next → &TX_DESC (set at init)
]);

/// RX link descriptor.  The `dst` (word 2) and `next` (word 7) fields are
/// patched at runtime by [`init`].
static mut RX_DESC: LinkDesc = LinkDesc([
    0b1101,                                    // Header
    (SSI0_BASE + SSIFRDR_OFF) as u32,          // src  = SSIFRDR (fixed)
    0,                                         // dst  (→ RX_BUF, set at init)
    (RX_BUF_LEN * core::mem::size_of::<i32>()) as u32, // transfer bytes
    RX_CHCFG,                                  // CHCFG
    0,                                         // CHITVL
    0,                                         // CHEXT
    0,                                         // next → &RX_DESC (set at init)
]);

// ── Register helpers ─────────────────────────────────────────────────────────

#[inline(always)]
fn ssi_reg(off: usize) -> *mut u32 {
    (SSI0_BASE + off) as *mut u32
}

// ── Public API ───────────────────────────────────────────────────────────────

/// Initialise SSI0 and start continuous DMA-driven I²S streaming.
///
/// After this call the DMA engine feeds the TX sample buffer into the codec
/// and captures the codec's output into the RX sample buffer, indefinitely.
///
/// **Call order requirements:**
/// 1. `rza1::stb::init()` — enables the SSIF0 module clock.
/// 2. Board audio pin-mux setup (done by `deluge_bsp::audio::init`).
/// 3. `ssi::init()` — this function.
/// 4. 5 ms hardware delay for codec power-on (done by `deluge_bsp::audio::init`).
/// 5. Assert the codec `CODEC_POWER` GPIO (done by `deluge_bsp::audio::init`).
///
/// # Safety
/// Must be called exactly once, from a single-threaded boot context, before
/// any audio task accesses the buffer pointers.
pub unsafe fn init() {
    log::debug!("ssi: init SSI0 (44.1 kHz I\u{00B2}S, DMA ch{}/{})", TX_DMA_CH, RX_DMA_CH);
    // 1. Patch self-referential fields in the link descriptors now that their
    //    static addresses are available.
    TX_DESC.0[1] = core::ptr::addr_of!(TX_BUF.0[0]) as u32;
    TX_DESC.0[7] = core::ptr::addr_of!(TX_DESC) as u32;

    RX_DESC.0[2] = core::ptr::addr_of!(RX_BUF.0[0]) as u32;
    RX_DESC.0[7] = core::ptr::addr_of!(RX_DESC) as u32;

    // 2. SSI0 software reset via CPG SWRSTCR1 bit 6.
    let swrstcr = SWRSTCR1 as *mut u8;
    let reset_bit = 1u8 << (6 - SSI_CH);
    swrstcr.write_volatile(swrstcr.read_volatile() | reset_bit);
    let _ = swrstcr.read_volatile(); // mandatory dummy read
    swrstcr.write_volatile(swrstcr.read_volatile() & !reset_bit);
    let _ = swrstcr.read_volatile(); // mandatory dummy read

    // 3. Configure SSI registers.
    ssi_reg(SSITDMR_OFF).write_volatile(0);       // no TDM
    ssi_reg(SSICR_OFF).write_volatile(SSICR_INIT); // master, 24-bit/32-bit, ÷4
    ssi_reg(SSIFCR_OFF).write_volatile(SSIFCR_INIT); // FIFOs in reset

    // 4. Initialise and connect DMA channels to the link descriptors.
    dmac::init_with_link_descriptor(
        TX_DMA_CH,
        core::ptr::addr_of!(TX_DESC) as *const u32,
        DMARS_SSI0_TX,
    );
    dmac::init_with_link_descriptor(
        RX_DMA_CH,
        core::ptr::addr_of!(RX_DESC) as *const u32,
        DMARS_SSI0_RX,
    );

    // 5. Start both DMA channels (software-reset then enable).
    dmac::channel_start(TX_DMA_CH);
    dmac::channel_start(RX_DMA_CH);
    log::debug!("ssi: DMA TX ch{} + RX ch{} started", TX_DMA_CH, RX_DMA_CH);

    // 6. Release the FIFOs from reset and enable TX/RX.
    //    This mirrors `ssiStart()` from the C firmware exactly.
    let fcr = ssi_reg(SSIFCR_OFF);

    // Release TX FIFO from reset (clear TFRST, bit 1)
    fcr.write_volatile(fcr.read_volatile() & !(1 << 1));
    // Enable TX-empty DMA trigger (set TIE, bit 3)
    fcr.write_volatile(fcr.read_volatile() | (1 << 3));
    // Release RX FIFO from reset (clear RFRST, bit 0)
    fcr.write_volatile(fcr.read_volatile() & !(1 << 0));
    // Enable RX-full DMA trigger (set RIE, bit 2)
    fcr.write_volatile(fcr.read_volatile() | (1 << 2));

    // Enable transmit and receive (TEN = bit 1, REN = bit 0 of SSICR)
    let cr = ssi_reg(SSICR_OFF);
    cr.write_volatile(cr.read_volatile() | 0b11);
    log::debug!("ssi: TX+RX enabled, streaming");
}

// ── Buffer pointer accessors ─────────────────────────────────────────────────
//
// All pointers are into the *uncached* mirror alias so the CPU never sees
// stale cache lines when reading or writing samples that the DMA has touched.

/// Current DMA read position in the TX buffer (uncached), aligned to one
/// stereo frame (8 bytes).
///
/// Write audio samples *ahead of* this pointer.  Wrap at [`tx_buf_end`].
pub fn tx_current_ptr() -> *mut i32 {
    let crsa = unsafe { dmac::current_src(TX_DMA_CH) };
    // Align down to a stereo frame (2 × i32 = 8 bytes)
    let aligned = crsa & !7u32;
    (aligned as usize + UNCACHED_MIRROR_OFFSET) as *mut i32
}

/// Current DMA write position in the RX buffer (uncached), aligned to one
/// stereo frame.
///
/// Read received samples *behind* this pointer.  Wrap at [`rx_buf_start`].
pub fn rx_current_ptr() -> *const i32 {
    let crda = unsafe { dmac::current_dst(RX_DMA_CH) };
    let aligned = crda & !7u32;
    (aligned as usize + UNCACHED_MIRROR_OFFSET) as *const i32
}

/// Pointer to the first sample in the TX buffer (uncached view).
pub fn tx_buf_start() -> *mut i32 {
    (unsafe { core::ptr::addr_of!(TX_BUF.0[0]) as usize } + UNCACHED_MIRROR_OFFSET) as *mut i32
}

/// One-past-the-end pointer for the TX buffer (uncached view).
pub fn tx_buf_end() -> *mut i32 {
    unsafe { tx_buf_start().add(TX_BUF_LEN) }
}

/// Pointer to the first sample in the RX buffer (uncached view).
pub fn rx_buf_start() -> *const i32 {
    (unsafe { core::ptr::addr_of!(RX_BUF.0[0]) as usize } + UNCACHED_MIRROR_OFFSET) as *const i32
}

/// One-past-the-end pointer for the RX buffer (uncached view).
pub fn rx_buf_end() -> *const i32 {
    unsafe { rx_buf_start().add(RX_BUF_LEN) }
}
