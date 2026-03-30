//! L1 and L2 cache initialisation for RZ/A1L (Cortex-A9 + PL310).
//!
//! Ports `R_CACHE_L1Init()` from `l1_cache_init.S` and `L2CacheInit()` from
//! `cache.c`.
//!
//! ## Ordering requirement
//! 1. [`mmu::init_and_enable`](crate::mmu::init_and_enable) must be called
//!    first so that the PL310 registers at `0x3FFF_F000` are accessible.
//! 2. Call [`l1_enable`] next — enables L1 I-cache, D-cache, branch
//!    prediction, and D-side prefetch.
//! 3. Call [`l2_init`] last — must come after [`l1_enable`] to avoid the L1
//!    flushing dirty data into a not-yet-valid L2.
//!
//! ## L2C (PL310) base address: 0x3FFF_F000
//! Register offsets follow the ARM PL310 TRM (r3p2):
//!
//! | Register          | Offset  |
//! |-------------------|---------|
//! | REG1_CONTROL      | 0x100   |
//! | REG2_INT_CLEAR    | 0x220   |
//! | REG7_INV_WAY      | 0x77C   |
//! | REG9_D_LOCKDOWN0  | 0x900   |
//! | REG9_I_LOCKDOWN0  | 0x904   |

use core::arch::asm;

// PL310 register addresses
const L2C_BASE:         usize = 0x3FFF_F000;
const L2C_REG1_CONTROL: usize = L2C_BASE + 0x100;
const L2C_REG2_INT_CLR: usize = L2C_BASE + 0x220;
const L2C_REG7_INV_WAY: usize = L2C_BASE + 0x77C;
const L2C_REG9_D_LOCK0: usize = L2C_BASE + 0x900;
const L2C_REG9_I_LOCK0: usize = L2C_BASE + 0x904;

const L2C_8WAY: u32 = 0x0000_00FF; // all 8 ways

// ---------------------------------------------------------------------------
// L1 cache
// ---------------------------------------------------------------------------

/// Enable Cortex-A9 L1 I-cache, D-cache, branch prediction, and D-side
/// prefetch.
///
/// Port of `R_CACHE_L1Init()` from `l1_cache_init.S`.
///
/// # Safety
/// Modifies CP15 `SCTLR` and `ACTLR`.  Must be called with the MMU already
/// enabled so that the cache attribute bits in the TTB are honoured.
pub unsafe fn l1_enable() {
    // Read SCTLR and set I-cache (bit 12), D-cache (bit 2), Z/branch (bit 11).
    let mut sctlr: u32;
    asm!("mrc p15, 0, {0}, c1, c0, 0", out(reg) sctlr, options(nomem, nostack));
    sctlr |= (1 << 12) | (1 << 11) | (1 << 2);
    asm!("mcr p15, 0, {0}, c1, c0, 0", in(reg) sctlr, options(nomem, nostack));
    asm!("isb", options(nomem, nostack));

    // Read ACTLR and set D-side prefetch (bit 2).
    let mut actlr: u32;
    asm!("mrc p15, 0, {0}, c1, c0, 1", out(reg) actlr, options(nomem, nostack));
    actlr |= 1 << 2;
    asm!("mcr p15, 0, {0}, c1, c0, 1", in(reg) actlr, options(nomem, nostack));
    asm!("isb", options(nomem, nostack));
}

// ---------------------------------------------------------------------------
// L2 cache (PL310)
// ---------------------------------------------------------------------------

/// Initialise and enable the PL310 L2 cache.
///
/// Port of `L2CacheInit()` from `cache.c`:
/// 1. Disable L2 cache.
/// 2. Flush all 8 ways (invalidate by way).
/// 3. Clear interrupt status.
/// 4. Lock D-cache ways (DMA bypass) and unlock I-cache ways.
/// 5. Enable L2 cache.
///
/// **Note:** Data-cache ways are locked (`0xFFFF_FFFF`) to prevent DMA
/// coherency issues — this matches the C BSP.  Instruction-cache ways are
/// left unlocked (fully caching).
///
/// # Safety
/// Writes to memory-mapped PL310 registers.  Must be called after
/// [`l1_enable`].
pub unsafe fn l2_init() {
    // 1. Disable L2 cache.
    wr32(L2C_REG1_CONTROL, 0x0000_0000);

    // 2. Flush all 8 ways — poll until hardware clears all bits.
    wr32(L2C_REG7_INV_WAY, L2C_8WAY);
    while rd32(L2C_REG7_INV_WAY) & L2C_8WAY != 0 {}

    // 3. Clear all interrupt sources in one write (bits [8:0]).
    wr32(L2C_REG2_INT_CLR, 0x0000_01FF);

    // 4. Lock D-cache ways (avoid DMA/cache coherency issues with DMA on L2).
    //    Unlock I-cache ways (allow instruction caching).
    wr32(L2C_REG9_D_LOCK0, 0xFFFF_FFFF);
    wr32(L2C_REG9_I_LOCK0, 0x0000_0000);

    // 5. Enable L2 cache.
    wr32(L2C_REG1_CONTROL, 0x0000_0001);
}

#[inline(always)]
unsafe fn wr32(addr: usize, val: u32) {
    core::ptr::write_volatile(addr as *mut u32, val);
}

#[inline(always)]
unsafe fn rd32(addr: usize) -> u32 {
    core::ptr::read_volatile(addr as *const u32)
}
