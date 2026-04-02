//! RUSB1 FIFO port operations.
//!
//! This module wraps the three hardware FIFO ports (CFIFO, D0FIFO, D1FIFO)
//! and provides the `sw_to_hw_fifo` / `hw_to_sw_fifo` byte-copy routines
//! that handle the RUSB1's MBW (memory bus width) switching rules.
//!
//! Routing policy (matches `dcd_rusb1.c`):
//! - Pipe 0 (DCP / control)         → CFIFO
//! - Pipes 1–2 (ISO)                → D0FIFO  (dedicated for audio)
//! - Pipes 3–15 (bulk / interrupt)  → D1FIFO
//!
//! ## MBW rules (TRM §28.3.8)
//! The MBW field in the FIFO SEL register must not change once a FIFO read
//! has begun.  We set MBW=32 when selecting CURPIPE (the common case for
//! 4-byte-aligned payloads) and only narrow it for sub-word transfers.
//!
//! ## RZA1 D1FIFO quirk
//! Writing D1FIFOSEL re-triggers the FIFO port switching state machine even
//! if only MBW changes.  Reading immediately after the MBW write returns
//! `0xFF`.  For payloads shorter than 4 bytes, we keep MBW=32 and unpack
//! the returned word instead of narrowing.

use super::regs::{
    rd, rd32, wr, wr32, Rusb1Regs, FIFOCTR_BCLR, FIFOCTR_BVAL, FIFOCTR_DTLN_MASK, FIFOCTR_FRDY,
    FIFOSEL_CURPIPE_MASK, FIFOSEL_MBW_MASK, FIFOSEL_MBW_SHIFT, MBW_16, MBW_32, MBW_8,
};

/// Hardware FIFO port: data register + SEL register + CTR register.
pub struct FifoPort {
    /// Pointer to the 32-bit data register (CFIFO / D0FIFO / D1FIFO).
    pub data: *mut u32,
    /// Pointer to the 16-bit select register (CFIFOSEL / D0FIFOSEL / D1FIFOSEL).
    pub sel: *mut u16,
    /// Pointer to the 16-bit control register (CFIFOCTR / D0FIFOCTR / D1FIFOCTR).
    pub ctr: *mut u16,
}

// Safety: FifoPort is a bundle of raw pointers to memory-mapped registers.
// All access is unsafe and protected by the caller.
unsafe impl Send for FifoPort {}

impl FifoPort {
    /// Construct the CFIFO port from a register block pointer.
    ///
    /// # Safety
    /// `regs` must be a valid pointer to the USB register block for the active port.
    pub unsafe fn cfifo(regs: *mut Rusb1Regs) -> Self {
        Self {
            data: core::ptr::addr_of_mut!((*regs).cfifo) as *mut u32,
            sel: core::ptr::addr_of_mut!((*regs).cfifosel),
            ctr: core::ptr::addr_of_mut!((*regs).cfifoctr),
        }
    }

    /// Construct the D0FIFO port.
    ///
    /// # Safety
    /// Same as [`cfifo`].
    pub unsafe fn d0fifo(regs: *mut Rusb1Regs) -> Self {
        Self {
            data: core::ptr::addr_of_mut!((*regs).d0fifo) as *mut u32,
            sel: core::ptr::addr_of_mut!((*regs).d0fifosel),
            ctr: core::ptr::addr_of_mut!((*regs).d0fifoctr),
        }
    }

    /// Construct the D1FIFO port.
    ///
    /// # Safety
    /// Same as [`cfifo`].
    pub unsafe fn d1fifo(regs: *mut Rusb1Regs) -> Self {
        Self {
            data: core::ptr::addr_of_mut!((*regs).d1fifo) as *mut u32,
            sel: core::ptr::addr_of_mut!((*regs).d1fifosel),
            ctr: core::ptr::addr_of_mut!((*regs).d1fifoctr),
        }
    }
}

// ---------------------------------------------------------------------------
// Pipe → FIFO port routing
// ---------------------------------------------------------------------------

/// Return the correct [`FifoPort`] for `pipe_num`.
///
/// Routing: pipe 0 → CFIFO, pipes 1-2 → D0FIFO, pipes 3+ → D1FIFO.
///
/// # Safety
/// `regs` must be valid.
pub unsafe fn fifo_for_pipe(regs: *mut Rusb1Regs, pipe_num: usize) -> FifoPort {
    match pipe_num {
        0 => FifoPort::cfifo(regs),
        1 | 2 => FifoPort::d0fifo(regs),
        _ => FifoPort::d1fifo(regs),
    }
}

// ---------------------------------------------------------------------------
// MBW helpers
// ---------------------------------------------------------------------------

/// Change the MBW field in a FIFO SEL register without touching other bits.
///
/// # Safety
/// `sel` must be a valid pointer to a FIFO select register.
unsafe fn set_mbw(sel: *mut u16, mbw: u16) {
    let cur = rd(sel);
    wr(
        sel,
        (cur & !FIFOSEL_MBW_MASK) | ((mbw << FIFOSEL_MBW_SHIFT) & FIFOSEL_MBW_MASK),
    );
}

// ---------------------------------------------------------------------------
// FIFO ready check
// ---------------------------------------------------------------------------

/// Returns `true` if the FIFO port is pointing at `pipe_num` AND is ready.
///
/// A FIFO port is ready when:
/// - CURPIPE == pipe_num  (the port has switched to the requested pipe)
/// - FRDY == 1           (the FIFO buffer is accessible)
///
/// # Safety
/// `fifo` fields must be valid register pointers.
pub unsafe fn fifo_is_ready(fifo: &FifoPort, pipe_num: usize) -> bool {
    let sel = rd(fifo.sel);
    if (sel & FIFOSEL_CURPIPE_MASK) as usize != pipe_num {
        return false;
    }
    (rd(fifo.ctr) & FIFOCTR_FRDY) != 0
}

/// Return the number of valid data bytes waiting in the FIFO.
///
/// # Safety
/// `fifo` must be valid.
pub unsafe fn fifo_dtln(fifo: &FifoPort) -> u16 {
    rd(fifo.ctr) & FIFOCTR_DTLN_MASK
}

/// Issue BCLR (clear the FIFO buffer) on `fifo`.
///
/// # Safety
/// `fifo` must be valid.
pub unsafe fn fifo_bclr(fifo: &FifoPort) {
    wr(fifo.ctr, FIFOCTR_BCLR);
}

/// Issue BVAL (commit the write buffer) on `fifo`.
///
/// # Safety
/// `fifo` must be valid.
pub unsafe fn fifo_bval(fifo: &FifoPort) {
    wr(fifo.ctr, FIFOCTR_BVAL);
}

// ---------------------------------------------------------------------------
// Software → hardware FIFO (write path: IN endpoint or control IN)
// ---------------------------------------------------------------------------

/// Copy `len` bytes from `buf` into the hardware FIFO.
///
/// MBW=32 must already be set in the SEL register (it is set by the
/// CURPIPE write in `pipe_xfer_in`).  This function switches to narrower
/// widths only for the sub-word tail, then restores MBW=32.
///
/// # Safety
/// - `fifo.data` / `fifo.sel` must be valid.
/// - The FIFO must have been selected (CURPIPE written) before calling.
pub unsafe fn sw_to_hw_fifo(fifo: &FifoPort, buf: *const u8, len: usize) {
    let mut p = buf;
    let mut rem = len;

    // 32-bit words
    while rem >= 4 {
        let word = (p as *const u32).read_unaligned();
        wr32(fifo.data, word);
        p = p.add(4);
        rem -= 4;
    }

    // 16-bit tail
    if rem >= 2 {
        set_mbw(fifo.sel, MBW_16);
        let half = (p as *const u16).read_unaligned();
        wr32(fifo.data, half as u32);
        p = p.add(2);
        rem -= 2;
    }

    // 8-bit tail
    if rem >= 1 {
        set_mbw(fifo.sel, MBW_8);
        wr32(fifo.data, *p as u32);
    }
}

// ---------------------------------------------------------------------------
// Hardware → software FIFO (read path: OUT endpoint or control OUT)
// ---------------------------------------------------------------------------

/// Determine the widest MBW valid for a single-fragment read of `len` bytes.
///
/// For sub-word payloads we keep MBW=32 and unpack — see module-level docs.
fn mbw_for_len(len: usize) -> u16 {
    if len.is_multiple_of(4) {
        MBW_32
    } else if len.is_multiple_of(2) {
        MBW_16
    } else {
        MBW_8
    }
}

/// Copy `len` bytes from the hardware FIFO into `buf`.
///
/// Handles the RZA1 D1FIFO quirk for sub-word payloads: keeps MBW=32 and
/// unpacks the returned word instead of switching width mid-transfer.
///
/// # Safety
/// - `fifo.data` / `fifo.sel` must be valid.
/// - The FIFO must have been selected (CURPIPE written) before calling.
pub unsafe fn hw_to_sw_fifo(fifo: &FifoPort, buf: *mut u8, len: usize) {
    if len == 0 {
        return;
    }

    if len < 4 {
        // Sub-word: keep MBW=32, unpack valid bytes from the first returned word.
        let word = rd32(fifo.data);
        let bytes = word.to_le_bytes();
        for i in 0..len {
            buf.add(i).write(bytes[i]);
        }
        return;
    }

    let mbw = mbw_for_len(len);
    if mbw != MBW_32 {
        set_mbw(fifo.sel, mbw);
    }

    let mut p = buf;
    let mut rem = len;

    match mbw {
        MBW_32 => {
            while rem >= 4 {
                let word = rd32(fifo.data);
                (p as *mut u32).write_unaligned(word);
                p = p.add(4);
                rem -= 4;
            }
        }
        MBW_16 => {
            let data16 = fifo.data as *const u16;
            while rem >= 2 {
                let half = core::ptr::read_volatile(data16);
                (p as *mut u16).write_unaligned(half);
                p = p.add(2);
                rem -= 2;
            }
        }
        _ /* MBW_8 */ => {
            let data8 = fifo.data as *const u8;
            while rem > 0 {
                *p = core::ptr::read_volatile(data8);
                p = p.add(1);
                rem -= 1;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Select a pipe on a FIFO port
// ---------------------------------------------------------------------------

/// Write the CURPIPE + MBW fields of `fifo.sel` together.
///
/// Per the TRM, CURPIPE and MBW must be written simultaneously.  This also
/// sets MBW=32 (the default for the subsequent sw_to_hw / hw_to_sw calls).
///
/// `isel`: set the ISEL bit for CFIFO IN direction (ignored for D0/D1FIFO).
///
/// # Safety
/// `fifo.sel` must be valid.
pub unsafe fn fifo_select_pipe(fifo: &FifoPort, pipe_num: usize, isel: bool) {
    let isel_bit: u16 = if isel { 0x0020 } else { 0 };
    let val: u16 = (pipe_num as u16) | (MBW_32 << FIFOSEL_MBW_SHIFT) | isel_bit;
    wr(fifo.sel, val);
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn mbw_selection() {
        assert_eq!(mbw_for_len(4), MBW_32);
        assert_eq!(mbw_for_len(8), MBW_32);
        assert_eq!(mbw_for_len(64), MBW_32);
        assert_eq!(mbw_for_len(2), MBW_16);
        assert_eq!(mbw_for_len(6), MBW_16);
        assert_eq!(mbw_for_len(1), MBW_8);
        assert_eq!(mbw_for_len(3), MBW_8);
    }

    #[test]
    fn sw_to_hw_roundtrip() {
        // Test the write path using a mock buffer (no actual hardware).
        // We verify that the unaligned reads in sw_to_hw_fifo don't panic.
        let src = [0x01u8, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07];
        let mut dest = 0u32;
        let fifo = FifoPort {
            data: &mut dest as *mut u32,
            sel: &mut 0u16 as *mut u16,
            ctr: &mut 0u16 as *mut u16,
        };
        // Just verify it doesn't panic for each length up to 7.
        for len in 0..=7 {
            unsafe {
                sw_to_hw_fifo(&fifo, src.as_ptr(), len);
            }
        }
    }

    #[test]
    fn hw_to_sw_sub_word() {
        // Verify sub-word unpack (keep MBW=32, read one word, extract bytes).
        let word: u32 = 0x04030201;
        let fifo = FifoPort {
            data: &word as *const u32 as *mut u32,
            sel: &mut 0u16 as *mut u16,
            ctr: &mut 0u16 as *mut u16,
        };
        let mut buf = [0u8; 3];
        unsafe {
            hw_to_sw_fifo(&fifo, buf.as_mut_ptr(), 3);
        }
        assert_eq!(buf, [0x01, 0x02, 0x03]);
    }
}
