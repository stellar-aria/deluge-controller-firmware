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
//!   4. UART  — async MIDI (UART0) and PIC32 (UART1) drivers
//!   5. SSI   — I2S audio codec at 44.1 kHz
//!   6. USB   — CDC/MIDI/Audio composite device

#![no_std]

pub mod gic;
pub mod ostm;

/// Bare-metal critical section for Cortex-A9: saves and restores the CPSR
/// I-bit (IRQ enable). Used by `rtt-target` and will be used by Embassy.
struct CortexACriticalSection;
critical_section::set_impl!(CortexACriticalSection);

unsafe impl critical_section::Impl for CortexACriticalSection {
    unsafe fn acquire() -> critical_section::RawRestoreState {
        let cpsr: u32;
        core::arch::asm!(
            "mrs {cpsr}, cpsr",
            "cpsid i",           // disable IRQ
            cpsr = out(reg) cpsr,
            options(nomem, nostack),
        );
        cpsr
    }

    unsafe fn release(cpsr: critical_section::RawRestoreState) {
        // Only re-enable IRQ if it was enabled before acquire
        core::arch::asm!(
            "msr cpsr_c, {cpsr}",
            cpsr = in(reg) cpsr,
            options(nomem, nostack),
        );
    }
}

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
}

/// UART (SCIF) registers (see RZ/A1L HW Manual §24).
pub mod uart {
    /// UART0 — MIDI DIN at 31250 baud (P6_3/P6_4)
    pub const UART0_BASE: usize = 0xE800_7000;
    /// UART1 — PIC32 button/encoder scanner at 200000 baud
    pub const UART1_BASE: usize = 0xE800_7800;

    pub const SCSMR:  usize = 0x00;
    pub const SCBRR:  usize = 0x04;
    pub const SCSCR:  usize = 0x08;
    pub const SCFTDR: usize = 0x0C;
    pub const SCFSR:  usize = 0x10;
    pub const SCFRDR: usize = 0x14;
    pub const SCFCR:  usize = 0x18;
}
