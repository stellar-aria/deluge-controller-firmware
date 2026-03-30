//! RZ/A1L peripheral register access.
//!
//! This crate will grow into the Embassy HAL for the Renesas RZ/A1L
//! (Cortex-A9, R7S721001). Peripheral base addresses are taken from the
//! RZ/A1L Hardware Manual.
//!
//! Milestone plan:
//!   1. GPIO  — SYNC LED toggle proof-of-life ✅
//!   2. GIC   — full interrupt infrastructure ✅
//!   3. OSTM  — free-running timer ✅ → Embassy `time_driver` impl
//!   4. UART  — async MIDI (UART0) and PIC32 (UART1) drivers ✅
//!   5. MMU + SDRAM — TTB, L1/L2 caches, BSC/SDRAM init ✅
//!   6. SSI   — I2S audio codec at 44.1 kHz
//!   7. USB   — CDC/MIDI/Audio composite device

#![cfg_attr(target_os = "none", no_std)]

// startup, mmu and cache use ARM coprocessor assembly and cannot compile on the host.
#[cfg(target_os = "none")]
pub mod startup;
#[cfg(target_os = "none")]
pub mod cache;
pub mod gic;
#[cfg(target_os = "none")]
pub mod mmu;
pub mod ostm;
pub mod sdram;
pub mod stb;
pub mod time_driver;
pub mod uart;

/// GPIO port registers (see RZ/A1L HW Manual §21).
///
/// Port numbering mirrors the hardware: passes 1..=11 for ports 1–11 (P1..P11).
/// PMC is the exception — it includes PMC0, so `pmc(0)` is valid.
///
/// Key register addresses (all 16-bit, stride 4):
/// - Pn    (output data)           : 0xFCFE_3004 + (n-1)*4
/// - PMn   (mode: 0=out, 1=in)     : 0xFCFE_3304 + (n-1)*4
/// - PMCn  (mux: 0=GPIO, 1=periph) : 0xFCFE_3400 + n*4
/// - PIPCn (ctrl: 0=sw, 1=periph)  : 0xFCFE_7204 + (n-1)*4
pub mod gpio {
    const GPIO_BASE: usize = 0xFCFE_3000;

    /// Port Mode Register n (PMn): bit=0 → output, bit=1 → input.
    /// `port` is 1-based (1..=11).
    pub fn pm(port: u8) -> *mut u16 {
        (GPIO_BASE + 0x300 + (port as usize) * 4) as *mut u16
    }

    /// Port Mode Control Register n (PMCn): bit=0 → GPIO, bit=1 → peripheral.
    /// `port` is 0-based (0..=11, PMC0 exists).
    pub fn pmc(port: u8) -> *mut u16 {
        (GPIO_BASE + 0x400 + (port as usize) * 4) as *mut u16
    }

    /// Port Data Register n (Pn) — output data.
    /// `port` is 1-based (1..=11).
    pub fn p(port: u8) -> *mut u16 {
        (GPIO_BASE + 0x000 + (port as usize) * 4) as *mut u16
    }

    /// Port Input Peripheral Control Register n (PIPCn): bit=0 → software ctrl.
    /// Must be 0 for GPIO output. `port` is 1-based (1..=11).
    /// Address: PIPC1 = 0xFCFE_7204, stride 4.
    pub fn pipc(port: u8) -> *mut u16 {
        (0xFCFE_7204usize + (port as usize - 1) * 4) as *mut u16
    }

    /// Configure a pin as a software-controlled GPIO output.
    ///
    /// # Safety
    /// Writes to memory-mapped peripheral registers; must run with the pin not
    /// already owned by another driver.
    pub unsafe fn set_as_output(port: u8, pin: u8) {
        // PMC = 0: GPIO (not peripheral multiplexed)
        let v = core::ptr::read_volatile(pmc(port));
        core::ptr::write_volatile(pmc(port), v & !(1u16 << pin));

        // PM = 0: output direction
        let v = core::ptr::read_volatile(pm(port));
        core::ptr::write_volatile(pm(port), v & !(1u16 << pin));

        // PIPC = 0: software (not hardware-peripheral) control
        let v = core::ptr::read_volatile(pipc(port));
        core::ptr::write_volatile(pipc(port), v & !(1u16 << pin));
    }

    /// Drive a GPIO output pin high (`true`) or low (`false`).
    ///
    /// # Safety
    /// The pin must have been configured as an output via [`set_as_output`].
    pub unsafe fn write(port: u8, pin: u8, high: bool) {
        let v = core::ptr::read_volatile(p(port));
        if high {
            core::ptr::write_volatile(p(port), v | (1u16 << pin));
        } else {
            core::ptr::write_volatile(p(port), v & !(1u16 << pin));
        }
    }

    /// Configure a pin as a peripheral function (GPIO pin-mux).
    ///
    /// Mirrors `setPinMux(port, pin, mux)` from `gpio.c`.
    ///
    /// `port`: 1-based (1–11).  `pin`: bit position (0–15).  `mux`: function (1–7).
    ///
    /// # Safety
    /// Writes to memory-mapped GPIO registers.
    pub unsafe fn set_pin_mux(port: u8, pin: u8, mux: u8) {
        // Base address for port 1 of each mux register group; stride 4 per port.
        const PFC_BASE:   usize = 0xFCFE_3504;
        const PFCE_BASE:  usize = 0xFCFE_3604;
        const PFCAE_BASE: usize = 0xFCFE_3A04;
        const PMC_BASE:   usize = 0xFCFE_3404; // PMC1 = 0x3404, PMCn = +4*(n-1)
        const PIPC_BASE:  usize = 0xFCFE_7204;

        fn modify(base: usize, port: u8, pin: u8, set: bool) {
            let addr = (base + (port as usize - 1) * 4) as *mut u16;
            let v = unsafe { core::ptr::read_volatile(addr) };
            let new = if set { v | (1u16 << pin) } else { v & !(1u16 << pin) };
            unsafe { core::ptr::write_volatile(addr, new) };
        }

        modify(PFCAE_BASE, port, pin, mux >= 5);
        modify(PFCE_BASE,  port, pin, ((mux - 1) >> 1) & 1 != 0);
        modify(PFC_BASE,   port, pin, (mux - 1) & 1 != 0);
        modify(PMC_BASE,   port, pin, true);
        modify(PIPC_BASE,  port, pin, true);
    }
}

#[cfg(test)]
mod tests {
    use super::gpio;

    // GPIO_BASE = 0xFCFE_3000
    // Pn   (output data)    = GPIO_BASE + 0x000 + port*4   (1-based port)
    // PMn  (mode)           = GPIO_BASE + 0x300 + port*4   (1-based port)
    // PMCn (mux enable)     = GPIO_BASE + 0x400 + port*4   (0-based port)
    // PIPCn (peripheral)    = 0xFCFE_7204 + (port-1)*4    (1-based port)

    #[test]
    fn gpio_p_addr() {
        assert_eq!(gpio::p(1) as usize,  0xFCFE_3004);
        assert_eq!(gpio::p(2) as usize,  0xFCFE_3008);
        assert_eq!(gpio::p(6) as usize,  0xFCFE_3018);
        assert_eq!(gpio::p(11) as usize, 0xFCFE_302C);
    }

    #[test]
    fn gpio_pm_addr() {
        assert_eq!(gpio::pm(1) as usize,  0xFCFE_3304);
        assert_eq!(gpio::pm(2) as usize,  0xFCFE_3308);
        assert_eq!(gpio::pm(6) as usize,  0xFCFE_3318);
        assert_eq!(gpio::pm(11) as usize, 0xFCFE_332C);
    }

    #[test]
    fn gpio_pmc_addr() {
        assert_eq!(gpio::pmc(0) as usize, 0xFCFE_3400);
        assert_eq!(gpio::pmc(1) as usize, 0xFCFE_3404);
        assert_eq!(gpio::pmc(6) as usize, 0xFCFE_3418);
        assert_eq!(gpio::pmc(11) as usize, 0xFCFE_342C);
    }

    #[test]
    fn gpio_pipc_addr() {
        assert_eq!(gpio::pipc(1) as usize, 0xFCFE_7204);
        assert_eq!(gpio::pipc(2) as usize, 0xFCFE_7208);
        assert_eq!(gpio::pipc(6) as usize, 0xFCFE_7218);
        assert_eq!(gpio::pipc(11) as usize, 0xFCFE_722C);
    }

    /// The {PFCAE, PFCE, PFC} 3-bit field encodes `mux - 1` for every valid
    /// mux function (1–7).  This matches the RZ/A1L HW Manual Table 21-x.
    #[test]
    fn gpio_pin_mux_encoding() {
        for mux in 1u8..=7 {
            let pfcae = mux >= 5;
            let pfce  = ((mux - 1) >> 1) & 1 != 0;
            let pfc   = (mux - 1) & 1 != 0;
            let bits  = ((pfcae as u8) << 2) | ((pfce as u8) << 1) | (pfc as u8);
            assert_eq!(bits, mux - 1, "mux={mux}");
        }
    }
}
