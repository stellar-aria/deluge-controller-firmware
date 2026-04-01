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
//! DMA Resource Selector (DMARS) registers are at `0xFCFE_A000 + (ch/2) * 4`.
//! Each 32-bit DMARS register covers a channel pair:
//!   - bits [15:0]  → even channel of the pair
//!   - bits [31:16] → odd  channel of the pair

const DMAC_BASE: usize = 0xE820_0000;
const DMARS_BASE: usize = 0xFCFE_A000;

// ── Channel register byte offsets within one `st_dmac_n` block ──────────────
const OFF_CRSA:   usize = 0x18; // Current source address
const OFF_CRDA:   usize = 0x1C; // Current destination address
const OFF_CHCTRL: usize = 0x28; // Channel control
const OFF_CHCFG:  usize = 0x2C; // Channel config
const OFF_NXLA:   usize = 0x38; // Next link-descriptor address

// ── CHCTRL bits ──────────────────────────────────────────────────────────────
const CHCTRL_SETEN: u32 = 1 << 0; // Set enable  (start transfer)
const CHCTRL_SWRST: u32 = 1 << 3; // Software reset (clears status)

// ── DCTRL group-control registers ───────────────────────────────────────────
const DCTRL_0_7:  usize = DMAC_BASE + 0x300;
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
    log::trace!("dmac: ch{} init, desc={:#010x}, dmars={:#06x}", ch, descriptor as usize, dmars_val);
    // 1. Clear group DCTRL (priority mode / round-robin reset)
    dctrl_reg(ch).write_volatile(0);

    // 2. CHCFG comes from link-descriptor word [4]
    ch_reg(ch, OFF_CHCFG).write_volatile(descriptor.add(4).read());

    // 3. DMARS: even channel → bits [15:0], odd channel → bits [31:16]
    let dmars = dmars_reg(ch);
    let (shifted, mask) = if ch & 1 == 0 {
        (dmars_val & 0xFFFF, 0xFFFF_0000u32)
    } else {
        ((dmars_val & 0xFFFF) << 16, 0x0000_FFFFu32)
    };
    dmars.write_volatile((dmars.read_volatile() & mask) | shifted);

    // 4. NXLA = address of the link descriptor
    ch_reg(ch, OFF_NXLA).write_volatile(descriptor as u32);
}

/// Start a DMA channel: software-reset then set-enable.
///
/// Mirrors `dmaChannelStart()` from the C firmware.
///
/// # Safety
/// The channel must have been initialised with [`init_with_link_descriptor`].
pub unsafe fn channel_start(ch: u8) {
    log::trace!("dmac: ch{} start (SWRST + SETEN)", ch);
    let chctrl = ch_reg(ch, OFF_CHCTRL);
    chctrl.write_volatile(chctrl.read_volatile() | CHCTRL_SWRST);
    chctrl.write_volatile(chctrl.read_volatile() | CHCTRL_SETEN);
}

/// Read the current DMA **source** address for channel `ch` (`CRSA_n`).
///
/// For a memory→peripheral transfer (TX), this is the SRAM read pointer.
///
/// # Safety
/// Reads a memory-mapped DMA register.
#[inline]
pub unsafe fn current_src(ch: u8) -> u32 {
    ch_reg(ch, OFF_CRSA).read_volatile()
}

/// Read the current DMA **destination** address for channel `ch` (`CRDA_n`).
///
/// For a peripheral→memory transfer (RX), this is the SRAM write pointer.
///
/// # Safety
/// Reads a memory-mapped DMA register.
#[inline]
pub unsafe fn current_dst(ch: u8) -> u32 {
    ch_reg(ch, OFF_CRDA).read_volatile()
}
