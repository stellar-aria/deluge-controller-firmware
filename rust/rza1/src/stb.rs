//! CPG Standby Control Register (STBCR) initialisation for RZ/A1L.
//!
//! Enables the module clocks for all peripherals used by the Deluge firmware
//! by writing CPG.STBCR2–STBCR12.  This is a direct port of `STB_Init()` in
//! `src/RZA1/stb/stb.c`.
//!
//! In CPG STBCRn, a **0 bit enables** (clock running) and a **1 bit stops**
//! (clock gated).  The values below are taken unchanged from the C BSP.
//!
//! ## CPG register addresses (RZ/A1L Hardware Manual §10)
//! CPG struct base: 0xFCFE_0010 (FRQCR).
//! Offsets derived from `cpg_iodefine.h` struct layout:
//!
//! | Register | Absolute address |
//! |----------|-----------------|
//! | STBCR2   | 0xFCFE_0024     |
//! | STBCR3   | 0xFCFE_0420     |
//! | STBCR4   | 0xFCFE_0424     |
//! | STBCR5   | 0xFCFE_0428     |
//! | STBCR6   | 0xFCFE_042C     |
//! | STBCR7   | 0xFCFE_0430     |
//! | STBCR8   | 0xFCFE_0434     |
//! | STBCR9   | 0xFCFE_0438     |
//! | STBCR10  | 0xFCFE_043C     |
//! | STBCR11  | 0xFCFE_0440     |
//! | STBCR12  | 0xFCFE_0444     |

const STBCR2:  usize = 0xFCFE_0024;
const STBCR3:  usize = 0xFCFE_0420;
const STBCR4:  usize = 0xFCFE_0424;
const STBCR5:  usize = 0xFCFE_0428;
const STBCR6:  usize = 0xFCFE_042C;
const STBCR7:  usize = 0xFCFE_0430;
const STBCR8:  usize = 0xFCFE_0434;
const STBCR9:  usize = 0xFCFE_0438;
const STBCR10: usize = 0xFCFE_043C;
const STBCR11: usize = 0xFCFE_0440;
const STBCR12: usize = 0xFCFE_0444;

// ---------------------------------------------------------------------------
// Per-register init values (0 bit = clock running, 1 bit = clock stopped).
// Named constants avoid magic literals and document which peripherals are
// enabled.  Bit order: bit 7 = leftmost column in the HW Manual bit diagram.
// Reserved bits that the C BSP holds at 1 are shown as underscores.
// ---------------------------------------------------------------------------

/// STBCR2: CoreSight enabled; port-level standby for unused bits.
/// `[1][1][0][1][0][1][CS][_]` → CoreSight on, others stopped.
const STBCR2_INIT: u8 = 0b01101010;

/// STBCR3: MTU2 and PWM enabled; IEBus/IrDA/LIN/RSCAN stopped.
/// `[IEBus][IrDA][LIN0][LIN1][MTU2][RSCAN2][0][PWM]`
/// 0 = running → MTU2 (bit 2) and PWM (bit 0) clocked.
const STBCR3_INIT: u8 = 0b11110101;

/// STBCR4: SCIF0–4 enabled; reserved bits [2:0] held at 1.
/// `[SCIF0][SCIF1][SCIF2][SCIF3][SCIF4][1][1][1]`
const STBCR4_INIT: u8 = 0b00000111;

/// STBCR5: OSTM0 and OSTM1 enabled; SCIM0/1 stopped.
/// `[SCIM0][SCIM1][1][1][1][1][OSTM1][OSTM0]`
/// Shared with `ostm::enable_clock()` which clears only bits [1:0].
const STBCR5_INIT: u8 = 0b11111100;

/// STBCR6: RTClock enabled; A/D, CEU, others stopped.
/// `[A/D][CEU][1][1][1][1][JCU][RTClock]`
const STBCR6_INIT: u8 = 0b01111111;

/// STBCR7: DVDEC0/1, ETHER, USB0, USB1 enabled; FLCTL stopped.
/// `[DVDEC0][DVDEC1][1][ETHER][FLCTL][1][USB0][USB1]`
const STBCR7_INIT: u8 = 0b00111101;

/// STBCR8: all stopped (IMR-LS2x, MMCIF, MOST50, SCUX unused).
const STBCR8_INIT: u8 = 0b11111111;

/// STBCR9: VDC50 enabled; I2Cx, SPIBSCx, VDC51 stopped.
/// `[I2C0][I2C1][I2C2][I2C3][SPIBSC0][SPIBSC1][VDC50][VDC51]`
const STBCR9_INIT: u8 = 0b11110111;

/// STBCR10: RSPI0–4 enabled; CD-ROMDEC, RSPDIF, RGPVG stopped.
/// `[RSPI0][RSPI1][RSPI2][RSPI3][RSPI4][CD-ROM][RSPDIF][RGPVG]`
const STBCR10_INIT: u8 = 0b00011111;

/// STBCR11: SSIF0–3 and SSIF5 enabled; SSIF4 stopped.
/// `[1][1][SSIF0][SSIF1][SSIF2][SSIF3][SSIF4][SSIF5]`
const STBCR11_INIT: u8 = 0b11011111;

/// STBCR12: SDHI0 channel 0 enabled (port 0, pins 0+1); SDHI1, SDHI0 ch1 stopped.
/// `[1][1][1][1][SDHI00][SDHI01][SDHI10][SDHI11]`
const STBCR12_INIT: u8 = 0b11111011;

/// Enable all peripheral module clocks.
///
/// Must be called before any peripheral (SCIF, OSTM, USB, SSIF, …) is
/// accessed.  The dummy-read after each write is required by hardware to
/// flush the write buffer before the next access (§10 of the HW manual).
///
/// # Safety
/// Writes to memory-mapped CPG registers.
pub unsafe fn init() {
    log::debug!("stb: enabling module clocks");
    // Port level is keep in standby mode, [1], [1], [0], [1], [0], [1], CoreSight
    wr8(STBCR2, STBCR2_INIT);
    let _: u8 = rd8(STBCR2); // dummy read

    // IEBus, IrDA, LIN0, LIN1, MTU2, RSCAN2, [0], PWM
    wr8(STBCR3, STBCR3_INIT);
    let _: u8 = rd8(STBCR3);

    // SCIF0, SCIF1, SCIF2, SCIF3, SCIF4, [1], [1], [1]  (0 = running)
    wr8(STBCR4, STBCR4_INIT);
    let _: u8 = rd8(STBCR4);

    // SCIM0, SCIM1, [1], [1], [1], [1], OSTM0, OSTM1
    wr8(STBCR5, STBCR5_INIT);
    let _: u8 = rd8(STBCR5);

    // A/D, CEU, [1], [1], [1], [1], JCU, RTClock
    wr8(STBCR6, STBCR6_INIT);
    let _: u8 = rd8(STBCR6);

    // DVDEC0, DVDEC1, [1], ETHER, FLCTL, [1], USB0, USB1
    wr8(STBCR7, STBCR7_INIT);
    let _: u8 = rd8(STBCR7);

    // IMR-LS20, IMR-LS21, IMR-LSD, MMCIF, MOST50, [1], SCUX, [1]
    wr8(STBCR8, STBCR8_INIT);
    let _: u8 = rd8(STBCR8);

    // I2C0, I2C1, I2C2, I2C3, SPIBSC0, SPIBSC1, VDC50, VDC51
    wr8(STBCR9, STBCR9_INIT);
    let _: u8 = rd8(STBCR9);

    // RSPI0, RSPI1, RSPI2, RSPI3, RSPI4, CD-ROMDEC, RSPDIF, RGPVG
    wr8(STBCR10, STBCR10_INIT);
    let _: u8 = rd8(STBCR10);

    // [1], [1], SSIF0, SSIF1, SSIF2, SSIF3, SSIF4, SSIF5
    wr8(STBCR11, STBCR11_INIT);
    let _: u8 = rd8(STBCR11);

    // [1], [1], [1], [1], SDHI00, SDHI01, SDHI10, SDHI11
    wr8(STBCR12, STBCR12_INIT);
    let _: u8 = rd8(STBCR12);
    log::debug!("stb: done (STBCR2-12 written)");
}

#[inline(always)]
unsafe fn wr8(addr: usize, val: u8) {
    core::ptr::write_volatile(addr as *mut u8, val);
}

#[inline(always)]
unsafe fn rd8(addr: usize) -> u8 {
    core::ptr::read_volatile(addr as *const u8)
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::{
        STBCR2, STBCR3, STBCR4, STBCR5, STBCR6, STBCR7,
        STBCR8, STBCR9, STBCR10, STBCR11, STBCR12,
    };

    /// Verify every STBCRn address against the RZ/A1L HW Manual §10 table.
    /// CPG base = 0xFCFE_0010 (FRQCR offset from module base 0xFCFE_0000).
    #[test]
    fn stbcr_register_addresses() {
        assert_eq!(STBCR2,  0xFCFE_0024);
        assert_eq!(STBCR3,  0xFCFE_0420);
        assert_eq!(STBCR4,  0xFCFE_0424);
        assert_eq!(STBCR5,  0xFCFE_0428);
        assert_eq!(STBCR6,  0xFCFE_042C);
        assert_eq!(STBCR7,  0xFCFE_0430);
        assert_eq!(STBCR8,  0xFCFE_0434);
        assert_eq!(STBCR9,  0xFCFE_0438);
        assert_eq!(STBCR10, 0xFCFE_043C);
        assert_eq!(STBCR11, 0xFCFE_0440);
        assert_eq!(STBCR12, 0xFCFE_0444);
    }

    /// STBCR3–STBCR12 are contiguous 4-byte-stride registers.
    #[test]
    fn stbcr3_to_stbcr12_are_contiguous() {
        let regs = [
            STBCR3, STBCR4, STBCR5, STBCR6, STBCR7,
            STBCR8, STBCR9, STBCR10, STBCR11, STBCR12,
        ];
        for window in regs.windows(2) {
            assert_eq!(window[1] - window[0], 4,
                "gap between {:08X} and {:08X} is not 4", window[0], window[1]);
        }
    }

    /// STBCR4 = 0b00000111 enables SCIF0–SCIF4 (bits 7–3 = 0) and keeps
    /// reserved bits 2–0 = 1.  SCIF5 (bit 2) does not exist on RZ/A1L.
    #[test]
    fn stbcr4_enables_all_scif() {
        // bits [7:3] = 0 → SCIF0–SCIF4 clocked; bits [2:0] = 1 → reserved/kept
        let val: u8 = 0b00000111;
        assert_eq!(val & 0b11111000, 0, "SCIF bits should all be 0 (enabled)");
    }

    /// STBCR5 = 0b11111100 enables OSTM0 (bit 0) and OSTM1 (bit 1).
    #[test]
    fn stbcr5_enables_ostm() {
        let val: u8 = 0b11111100;
        assert_eq!(val & 0x03, 0, "OSTM0/OSTM1 bits should be 0 (enabled)");
    }
}
