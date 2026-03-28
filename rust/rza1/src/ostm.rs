//! OS Timer (OSTM) driver for the RZ/A1L.
//!
//! The RZ/A1L has two OSTM channels, each a 32-bit timer clocked by P0
//! (33.33 MHz = CPU / 12).
//!
//! Two operating modes (OSTMnCTL bits):
//!   - Interval timer (CTL = 0x00): counts down from CMP, fires interrupt at 0
//!   - Free-running  (CTL = 0x02): counts up from 0, optionally fires at CMP
//!
//! ## Usage
//! ```rust
//! unsafe {
//!     ostm::enable_clock();          // ungate OSTM0+OSTM1 in CPG
//!     ostm::start_free_running(0);   // start OSTM0 in free-running mode
//!     let t0 = ostm::count(0);
//!     // ... wait ~1 s ...
//!     while ostm::count(0).wrapping_sub(t0) < OSTM_HZ {}
//! }
//! ```

/// OSTM P0 clock frequency (Hz). P0 = CPU / 12 = 400 MHz / 12 = 33.333... MHz.
pub const OSTM_HZ: u32 = 33_333_333;

// ------- Peripheral base addresses -------------------------------------------
const OSTM0_BASE: usize = 0xFCFE_C000;
const OSTM1_BASE: usize = 0xFCFE_C400;

// ------- Register offsets (from OSTMn base) ----------------------------------
const CMP: usize = 0x00; // OSTMnCMP — compare register (32-bit)
const CNT: usize = 0x04; // OSTMnCNT — counter, read-only (32-bit)
const TE:  usize = 0x10; // OSTMnTE  — timer enable status (8-bit, read-only)
const TS:  usize = 0x14; // OSTMnTS  — timer start: write 1 to start (8-bit)
const TT:  usize = 0x18; // OSTMnTT  — timer stop:  write 1 to stop  (8-bit)
const CTL: usize = 0x20; // OSTMnCTL — control: bit1=mode, bit0=int_enable (8-bit)

// CTL mode values
const CTL_INTERVAL:    u8 = 0x00; // count-down interval timer, optional interrupt
const CTL_FREE_RUN:    u8 = 0x02; // free-running counter up, optional interrupt
const CTL_FREE_RUN_INT: u8 = 0x03; // free-running with compare interrupt

// ------- CPG standby control (clock gating) ----------------------------------
//
// CPG base = 0xFCFE_0010. STBCR5 is at offset 0x418 from the base:
//   STBCR5 address = 0xFCFE_0010 + 0x418 = 0xFCFE_0428
//
// STBCR5 layout (from stb.c comment):
//   bit 7 : SCIM0  stop (1 = stopped)
//   bit 6 : SCIM1  stop
//   bit 5 : [reserved, keep 1]
//   bit 4 : [reserved, keep 1]
//   bit 3 : [reserved, keep 1]
//   bit 2 : [reserved, keep 1]
//   bit 1 : OSTM1  stop  ← MSTP51
//   bit 0 : OSTM0  stop  ← MSTP50
//
// stb.c initializes STBCR5 = 0b11111100 — bits 0 and 1 clear → OSTM0+OSTM1 clocked.
// A dummy read after write is required by the hardware spec.
const STBCR5: usize = 0xFCFE_0428;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Ungate OSTM0 and OSTM1 clocks in the CPG module stop register (STBCR5).
///
/// The Renesas `STB_Init()` normally sets `STBCR5 = 0b11111100` which already
/// has bits 0+1 clear (= OSTM0+OSTM1 enabled). This function clears only
/// those two bits so it is safe to call even if the rest of STBCR5 was set
/// by an earlier boot stage.
///
/// # Safety
/// Writes to memory-mapped CPG registers. Safe to call once before starting
/// any OSTM channel.
pub unsafe fn enable_clock() {
    let reg = STBCR5 as *mut u8;
    let val = reg.read_volatile();
    reg.write_volatile(val & !0x03); // clear MSTP50 (OSTM0) and MSTP51 (OSTM1)
    let _ = reg.read_volatile(); // mandatory dummy read per HW spec
}

#[inline(always)]
fn base(n: u8) -> usize {
    if n == 0 { OSTM0_BASE } else { OSTM1_BASE }
}

/// Configure OSTM channel `n` (0 or 1) for free-running mode and start it.
///
/// The counter begins at 0 and increments at P0 (~33.33 MHz). No interrupt
/// is generated. Safe to call when the channel is already stopped; if it is
/// running it is stopped first to allow CTL to be written.
///
/// # Safety
/// Writes to memory-mapped OSTM registers. Must only be called after
/// [`enable_clock`].
pub unsafe fn start_free_running(n: u8) {
    let b = base(n);
    // Stop the timer in case it is already running (CTL can only be written
    // when the timer is stopped).
    let tt = (b + TT) as *mut u8;
    tt.write_volatile(1);

    // Free-running mode, compare interrupt disabled.
    let ctl = (b + CTL) as *mut u8;
    ctl.write_volatile(CTL_FREE_RUN);

    // Start.
    let ts = (b + TS) as *mut u8;
    ts.write_volatile(1);
}

/// Read the current counter value of OSTM channel `n` (0 or 1).
///
/// Increments at ~33.33 MHz. Wraps at `u32::MAX`.
///
/// # Safety
/// Reads from memory-mapped OSTM registers.
#[inline]
pub unsafe fn count(n: u8) -> u32 {
    let cnt = (base(n) + CNT) as *const u32;
    cnt.read_volatile()
}

/// Busy-wait for approximately `ticks` OSTM counts (~30 ns each at P0).
///
/// Uses OSTM channel 0. [`start_free_running`]`(0)` must have been called first.
///
/// # Safety
/// Reads from memory-mapped OSTM registers.
pub unsafe fn delay_ticks(ticks: u32) {
    let t0 = count(0);
    while count(0).wrapping_sub(t0) < ticks {}
}

/// Busy-wait for approximately `ms` milliseconds using OSTM channel 0.
///
/// # Safety
/// See [`delay_ticks`].
pub unsafe fn delay_ms(ms: u32) {
    // OSTM_HZ / 1000 = 33_333 ticks per millisecond
    delay_ticks(ms * (OSTM_HZ / 1000));
}
