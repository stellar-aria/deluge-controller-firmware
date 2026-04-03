//! Renesas RZ/A1L DMA Controller (DMAC) low-level register driver.
//!
//! The DMAC has 16 channels (0–15), split into two groups:
//!   - Channels  0–7: registers at `0xE820_0000 + ch * 64`
//!   - Channels 8–15: registers at `0xE820_0000 + ch * 64 + 0x200`
//!
//! Each channel has 16 × 32-bit registers; the stride between channels is
//! exactly 64 bytes (16 × 4).
//!
//! Group control registers:
//!   - `DCTRL_0_7`  at `0xE820_0300`
//!   - `DCTRL_8_15` at `0xE820_0700`
//!
//! DMA Resource Selector (DMARS) registers are at `0xFCFE_1000 + (ch/2) * 4`.
//! Each 32-bit DMARS register covers a channel pair:
//!   - bits [15:0]  → even channel of the pair
//!   - bits [31:16] → odd  channel of the pair
//!
//! (RZ/A1L TRM §9, register table: DMARS0=0xFCFE1000 … DMARS7=0xFCFE101C)

const DMAC_BASE: usize = 0xE820_0000;
const DMARS_BASE: usize = 0xFCFE_1000;

// ── Channel register byte offsets within one `st_dmac_n` block ──────────────
// Register-mode (N0/N1) source, destination, and byte-count registers.
const OFF_N0SA: usize = 0x00; // Next-0 source address
const OFF_N0DA: usize = 0x04; // Next-0 destination address
const OFF_N0TB: usize = 0x08; // Next-0 transfer byte count
const OFF_CRSA: usize = 0x18; // Current source address
const OFF_CRDA: usize = 0x1C; // Current destination address
const OFF_CHCTRL: usize = 0x28; // Channel control
const OFF_CHCFG: usize = 0x2C; // Channel config
const OFF_CHITVL: usize = 0x30; // Channel interval
const OFF_CHEXT: usize = 0x34; // Channel extension
const OFF_NXLA: usize = 0x38; // Next link-descriptor address

// ── CHCTRL bits ──────────────────────────────────────────────────────────────
const CHCTRL_SETEN: u32 = 1 << 0; // Set enable  (start transfer)
const CHCTRL_SWRST: u32 = 1 << 3; // Software reset (clears status)
const CHCTRL_CLRTC: u32 = 1 << 6; // Clear terminal count (TC bit)

// ── DCTRL group-control registers ───────────────────────────────────────────
const DCTRL_0_7: usize = DMAC_BASE + 0x300;
const DCTRL_8_15: usize = DMAC_BASE + 0x700;

// ── Helpers ──────────────────────────────────────────────────────────────────

#[inline]
fn ch_base(ch: u8) -> usize {
    DMAC_BASE + (ch as usize) * 64 + if ch >= 8 { 0x200 } else { 0 }
}

#[inline]
fn ch_reg(ch: u8, off: usize) -> *mut u32 {
    (ch_base(ch) + off) as *mut u32
}

#[inline]
fn dctrl_reg(ch: u8) -> *mut u32 {
    (if ch < 8 { DCTRL_0_7 } else { DCTRL_8_15 }) as *mut u32
}

#[inline]
fn dmars_reg(ch: u8) -> *mut u32 {
    (DMARS_BASE + (ch as usize / 2) * 4) as *mut u32
}

// ── Public API ───────────────────────────────────────────────────────────────

/// Initialise a DMA channel in link-descriptor mode.
///
/// Mirrors `initDMAWithLinkDescriptor()` from the C firmware:
/// 1. Clears DCTRL for the channel's group.
/// 2. Loads `CHCFG` from word \[4\] of the descriptor.
/// 3. Programs the DMARS resource selector.
/// 4. Writes the descriptor pointer to `NXLA`.
///
/// # Safety
/// - `descriptor` must point to a valid, 32-byte-aligned 8-word link
///   descriptor whose lifetime extends past the end of the DMA operation.
/// - Must be called before [`channel_start`] for the same channel.
pub unsafe fn init_with_link_descriptor(ch: u8, descriptor: *const u32, dmars_val: u32) {
    unsafe {
        log::trace!(
            "dmac: ch{} init, desc={:#010x}, dmars={:#06x}",
            ch,
            descriptor as usize,
            dmars_val
        );
        // 1. Clear group DCTRL (priority mode / round-robin reset)
        log::trace!(
            "dmac: ch{} writing DCTRL={:#010x}",
            ch,
            dctrl_reg(ch) as usize
        );
        dctrl_reg(ch).write_volatile(0);
        log::trace!("dmac: ch{} DCTRL ok", ch);

        // 2. CHCFG comes from link-descriptor word [4]
        let chcfg_val = descriptor.add(4).read();
        log::trace!(
            "dmac: ch{} desc[4]={:#010x}, writing CHCFG={:#010x}",
            ch,
            chcfg_val,
            ch_reg(ch, OFF_CHCFG) as usize
        );
        ch_reg(ch, OFF_CHCFG).write_volatile(chcfg_val);
        log::trace!("dmac: ch{} CHCFG ok", ch);

        // 3. DMARS: even channel → bits [15:0], odd channel → bits [31:16]
        let dmars = dmars_reg(ch);
        let (shifted, mask) = if ch & 1 == 0 {
            (dmars_val & 0xFFFF, 0xFFFF_0000u32)
        } else {
            ((dmars_val & 0xFFFF) << 16, 0x0000_FFFFu32)
        };
        log::trace!("dmac: ch{} writing DMARS={:#010x}", ch, dmars as usize);
        dmars.write_volatile((dmars.read_volatile() & mask) | shifted);
        log::trace!("dmac: ch{} DMARS ok", ch);

        // 4. NXLA = address of the link descriptor
        log::trace!(
            "dmac: ch{} writing NXLA={:#010x}",
            ch,
            ch_reg(ch, OFF_NXLA) as usize
        );
        ch_reg(ch, OFF_NXLA).write_volatile(descriptor as u32);
        log::trace!("dmac: ch{} NXLA ok", ch);
    }
}

/// Start a DMA channel: software-reset then set-enable.
///
/// Mirrors `dmaChannelStart()` from the C firmware.
///
/// # Safety
/// The channel must have been initialised with [`init_with_link_descriptor`].
pub unsafe fn channel_start(ch: u8) {
    unsafe {
        log::trace!("dmac: ch{} start (SWRST + SETEN)", ch);
        let chctrl = ch_reg(ch, OFF_CHCTRL);
        chctrl.write_volatile(chctrl.read_volatile() | CHCTRL_SWRST);
        chctrl.write_volatile(chctrl.read_volatile() | CHCTRL_SETEN);
    }
}

/// Read the current DMA **source** address for channel `ch` (`CRSA_n`).
///
/// For a memory→peripheral transfer (TX), this is the SRAM read pointer.
///
/// # Safety
/// Reads a memory-mapped DMA register.
#[inline]
pub unsafe fn current_src(ch: u8) -> u32 {
    unsafe { ch_reg(ch, OFF_CRSA).read_volatile() }
}

/// Read the current DMA **destination** address for channel `ch` (`CRDA_n`).
///
/// For a peripheral→memory transfer (RX), this is the SRAM write pointer.
///
/// # Safety
/// Reads a memory-mapped DMA register.
#[inline]
pub unsafe fn current_dst(ch: u8) -> u32 {
    unsafe { ch_reg(ch, OFF_CRDA).read_volatile() }
}

// ── Register-mode (one-shot) DMA ─────────────────────────────────────────────
//
// Used for memory→peripheral block transfers where the transfer size is known
// in advance (e.g. OLED SPI TX).  Unlike link-descriptor mode, the channel
// stops after one transfer and can be re-armed by calling `start_transfer`
// again.  Completion is signalled via a GIC interrupt (DMAINT_n, GIC IDs
// 41–56 for channels 0–15).

use core::future::poll_fn;
use core::task::Poll;
use embassy_sync::waitqueue::AtomicWaker;

/// GIC interrupt ID base for DMAC completion interrupts: DMAINT0 = 41.
const DMAINT_BASE: u16 = 41;

/// Per-channel waker for DMA completion interrupts.
static DMAC_WAKERS: [AtomicWaker; 16] = [
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
];

/// Initialise a DMAC channel in **register mode** for a memory→peripheral
/// block transfer.
///
/// Mirrors `oledDMAInit()` from the C firmware:
/// 1. Clears `DCTRL` for the channel's group.
/// 2. Programs `CHCFG`, `CHITVL`, `CHEXT`.
/// 3. Writes the fixed destination address (`N0DA`) — e.g. peripheral data register.
/// 4. Software-resets the channel (clears `TC`).
/// 5. Programs the DMARS resource selector.
///
/// The channel is left idle.  Call [`start_transfer`] for each block.  Call
/// [`register_completion_irq`] separately to receive a GIC interrupt on
/// transfer end.
///
/// # Safety
/// Writes to DMAC registers.  Must be called before the channel is used.
pub unsafe fn init_register_mode(ch: u8, chcfg: u32, dst: u32, dmars: u32) {
    unsafe {
        log::debug!(
            "dmac: ch{} register-mode init, chcfg={:#010x}, dst={:#010x}, dmars={:#06x}",
            ch,
            chcfg,
            dst,
            dmars
        );

        // 1. Clear group DCTRL.
        dctrl_reg(ch).write_volatile(0);

        // 2. CHCFG / CHITVL / CHEXT.
        ch_reg(ch, OFF_CHCFG).write_volatile(chcfg);
        ch_reg(ch, OFF_CHITVL).write_volatile(0);
        ch_reg(ch, OFF_CHEXT).write_volatile(0);

        // 3. Fixed destination (peripheral register).
        ch_reg(ch, OFF_N0DA).write_volatile(dst);

        // 4. Software reset + clear TC.
        let chctrl = ch_reg(ch, OFF_CHCTRL);
        chctrl.write_volatile(chctrl.read_volatile() | CHCTRL_SWRST | CHCTRL_CLRTC);
        // Re-write N0DA after reset (matches C firmware's double-write pattern).
        ch_reg(ch, OFF_N0DA).write_volatile(dst);

        // 5. DMARS.
        let dmars_ptr = dmars_reg(ch);
        let (shifted, mask) = if ch & 1 == 0 {
            (dmars & 0xFFFF, 0xFFFF_0000u32)
        } else {
            ((dmars & 0xFFFF) << 16, 0x0000_FFFFu32)
        };
        dmars_ptr.write_volatile((dmars_ptr.read_volatile() & mask) | shifted);
    }
}

/// Register the DMAC completion (DMAINT) GIC interrupt for channel `ch`.
///
/// The ISR calls [`on_dma_int`] which wakes any task awaiting
/// [`wait_transfer_complete`].
///
/// # Safety
/// Writes to GIC registers. Call after [`crate::gic::init`] and before IRQs
/// are enabled.
pub unsafe fn register_completion_irq(ch: u8) {
    unsafe {
        use crate::gic;
        let id = DMAINT_BASE + ch as u16;
        gic::register(id, DMA_INT_HANDLERS[ch as usize]);
        gic::set_priority(id, 13); // matches original C firmware priority for OLED DMA
        gic::enable(id);
    }
}

/// Call from the DMAIC completion ISR for channel `ch`.
///
/// Wakes any Embassy task waiting in [`wait_transfer_complete`].
pub fn on_dma_int(ch: u8) {
    DMAC_WAKERS[ch as usize].wake();
}

/// Arm and start a one-shot DMA transfer on a channel set up with
/// [`init_register_mode`].
///
/// Sets `N0SA = src`, `N0TB = count`, then writes `CLRTC | SETEN` to kick
/// the channel.  Matches the per-frame part of `oledSelectingComplete()`.
///
/// # Safety
/// The source data at `[src, src+count)` must be in uncached memory (or have
/// had its cache lines flushed) so the DMAC reads the correct bytes.
pub unsafe fn start_transfer(ch: u8, src: u32, count: u32) {
    unsafe {
        log::trace!(
            "dmac: ch{} start_transfer src={:#010x} count={}",
            ch,
            src,
            count
        );
        ch_reg(ch, OFF_N0SA).write_volatile(src);
        ch_reg(ch, OFF_N0TB).write_volatile(count);
        let chctrl = ch_reg(ch, OFF_CHCTRL);
        chctrl.write_volatile(chctrl.read_volatile() | CHCTRL_CLRTC | CHCTRL_SETEN);
    }
}

/// Suspend until the DMA transfer on channel `ch` completes.
///
/// Completion is signalled by the DMAINT GIC interrupt (registered via
/// [`register_completion_irq`]).  This function must be called **after**
/// [`start_transfer`] has kicked the channel.
pub async fn wait_transfer_complete(ch: u8) {
    poll_fn(|cx| {
        DMAC_WAKERS[ch as usize].register(cx.waker());
        // Check C-flag (CHSTAT.TC) directly to avoid missing a completion
        // that arrived before we registered the waker.
        let chstat = unsafe { ch_reg(ch, 0x24).read_volatile() };
        if chstat & (1 << 6) != 0 {
            // TC bit set — transfer complete
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
    .await
}

// Per-channel ISR dispatch table (16 entries; only populated channels matter).
fn dma_int_0() {
    on_dma_int(0);
}
fn dma_int_1() {
    on_dma_int(1);
}
fn dma_int_2() {
    on_dma_int(2);
}
fn dma_int_3() {
    on_dma_int(3);
}
fn dma_int_4() {
    on_dma_int(4);
}
fn dma_int_5() {
    on_dma_int(5);
}
fn dma_int_6() {
    on_dma_int(6);
}
fn dma_int_7() {
    on_dma_int(7);
}
fn dma_int_8() {
    on_dma_int(8);
}
fn dma_int_9() {
    on_dma_int(9);
}
fn dma_int_10() {
    on_dma_int(10);
}
fn dma_int_11() {
    on_dma_int(11);
}
fn dma_int_12() {
    on_dma_int(12);
}
fn dma_int_13() {
    on_dma_int(13);
}
fn dma_int_14() {
    on_dma_int(14);
}
fn dma_int_15() {
    on_dma_int(15);
}

type HandlerFn = fn();
static DMA_INT_HANDLERS: [HandlerFn; 16] = [
    dma_int_0, dma_int_1, dma_int_2, dma_int_3, dma_int_4, dma_int_5, dma_int_6, dma_int_7,
    dma_int_8, dma_int_9, dma_int_10, dma_int_11, dma_int_12, dma_int_13, dma_int_14, dma_int_15,
];
