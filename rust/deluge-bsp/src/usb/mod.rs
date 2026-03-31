//! RUSB1 USB driver — public module façade.
//!
//! ## Quick start — device mode
//!
//! ```rust,no_run
//! use deluge_bsp::usb::{init_device_mode, UsbMode};
//!
//! let (port, driver) = unsafe { init_device_mode(0) };
//! // pass `driver` to `embassy_usb::UsbDevice::new(driver, config, ...)`
//! ```
//!
//! ## Quick start — host mode (pending Embassy PR #5633)
//!
//! ```rust,no_run
//! use deluge_bsp::usb::init_host_mode;
//!
//! let (port, host) = unsafe { init_host_mode(0) };
//! // `host` will implement `UsbHostDriver` once PR #5633 merges
//! ```
//!
//! ## ISR wiring
//!
//! You must call `dcd_int_handler` from your GIC interrupt dispatcher:
//!
//! ```rust,no_run
//! use deluge_bsp::usb::dcd_int_handler;
//!
//! #[no_mangle]
//! extern "C" fn irq73_handler() {  // USB0
//!     unsafe { dcd_int_handler(0); }
//! }
//! #[no_mangle]
//! extern "C" fn irq74_handler() {  // USB1
//!     unsafe { dcd_int_handler(1); }
//! }
//! ```

pub mod regs;
pub mod fifo;
pub mod pipe;
pub mod driver;
pub mod host;
pub mod classes;

pub use driver::{
    Rusb1Driver, Rusb1Bus, Rusb1ControlPipe,
    Rusb1EndpointIn, Rusb1EndpointOut,
    dcd_int_handler,
};
pub use host::Rusb1HostDriver;

use core::marker::PhantomData;
use rza1::rusb1;

// ---------------------------------------------------------------------------
// Mode markers
// ---------------------------------------------------------------------------

/// Mode marker for device (peripheral) operation.
pub struct Device;

/// Mode marker for host operation.
pub struct Host;

// ---------------------------------------------------------------------------
// UsbPort handle
// ---------------------------------------------------------------------------

/// A handle representing ownership of one RUSB1 port in a given `MODE`.
///
/// Created by [`init_device_mode`] or [`init_host_mode`]; not constructible
/// directly.  Dropping this type does not disable the hardware — call
/// [`UsbPort::into_device_mode`] or [`UsbPort::into_host_mode`] explicitly to
/// switch modes at runtime.
pub struct UsbPort<MODE> {
    port: u8,
    _mode: PhantomData<MODE>,
}

impl<M> UsbPort<M> {
    /// The hardware port index (0 or 1).
    pub fn port_index(&self) -> u8 { self.port }
}

impl UsbPort<Device> {
    /// Re-enter host mode.  Disables pull-up, reconfigures DCFM, re-enables.
    ///
    /// # Safety
    /// No active USB traffic must be in progress.
    pub unsafe fn into_host_mode(self) -> (UsbPort<Host>, Rusb1HostDriver) {
        quiesce_port(self.port);
        let hd = Rusb1HostDriver::new(self.port);
        (UsbPort { port: self.port, _mode: PhantomData }, hd)
    }
}

impl UsbPort<Host> {
    /// Re-enter device mode.
    ///
    /// # Safety
    /// No active USB traffic must be in progress.
    pub unsafe fn into_device_mode(self) -> (UsbPort<Device>, Rusb1Driver) {
        quiesce_port(self.port);
        let drv = Rusb1Driver::new(self.port);
        (UsbPort { port: self.port, _mode: PhantomData }, drv)
    }
}

// ---------------------------------------------------------------------------
// Convenience initialisation functions
// ---------------------------------------------------------------------------

/// Enable the USB clock, initialise the RUSB1 hardware in **device** mode,
/// and return an ownership handle plus the `embassy-usb-driver` driver.
///
/// Call once per port; typically at startup before `embassy_usb::UsbDevice`
/// is created.
///
/// # Safety
/// Must only be called once per port.  Caller must ensure the PLL (UPLLE) and
/// the USB clock select (UCKSEL) have been configured in SYSCFG0 before this
/// call if an external clock is used (the Deluge board uses the internal PLL).
pub unsafe fn init_device_mode(port: u8) -> (UsbPort<Device>, Rusb1Driver) {
    rusb1::module_clock_enable(port);
    let drv = Rusb1Driver::new(port);
    (UsbPort { port, _mode: PhantomData }, drv)
}

/// Enable the USB clock, initialise the RUSB1 hardware in **host** mode, and
/// return an ownership handle plus the host driver stub.
///
/// ## Warning — stub implementation
/// The returned `Rusb1HostDriver` is not yet functional.  See
/// `deluge-bsp/src/usb/host.rs` for the implementation roadmap.
///
/// # Safety
/// Must only be called once per port.
pub unsafe fn init_host_mode(port: u8) -> (UsbPort<Host>, Rusb1HostDriver) {
    rusb1::module_clock_enable(port);
    let hd = Rusb1HostDriver::new(port);
    (UsbPort { port, _mode: PhantomData }, hd)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Bring a port to a quiescent state (USBE=0, interrupts off) before a mode
/// switch.
unsafe fn quiesce_port(port: u8) {
    use regs::{Rusb1Regs, SYSCFG_USBE, wr};
    rusb1::int_disable(port);
    let regs = Rusb1Regs::ptr(port);
    wr(core::ptr::addr_of_mut!((*regs).intenb0), 0);
    wr(core::ptr::addr_of_mut!((*regs).brdyenb), 0);
    wr(core::ptr::addr_of_mut!((*regs).bempenb), 0);
    // Clear USBE to reset the SIE.
    let cur = regs::rd(core::ptr::addr_of!((*regs).syscfg0));
    wr(core::ptr::addr_of_mut!((*regs).syscfg0), cur & !SYSCFG_USBE);
}
