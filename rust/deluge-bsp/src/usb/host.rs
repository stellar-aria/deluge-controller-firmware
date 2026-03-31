//! RUSB1 USB host-mode driver skeleton.
//!
//! ## Status — INCOMPLETE / PLACEHOLDER
//!
//! Full host-mode implementation is deferred until the Embassy
//! `UsbHostDriver` + `UsbChannel` traits stabilise and merge into the main
//! `embassy-rs/embassy` repository.
//!
//! **Tracking issue**: <https://github.com/embassy-rs/embassy/pull/5633>
//!
//! The structs and impl stubs below establish the correct API surface so that
//! code that conditionally uses `-F usb-host` compiles without errors.
//! Every method body contains `todo!()` and will panic at runtime; no host
//! functionality is usable until the TODO comments are resolved.
//!
//! ## How to resume this implementation
//!
//! 1. Wait for Embassy PR #5633 to merge (or rebase onto it).
//! 2. Add the following to `deluge-bsp/Cargo.toml` under the
//!    `[target.'cfg(target_os = "none")'.dependencies]` section:
//!    ```toml
//!    embassy-usb-host = { version = "0.1" }  # or git dep if not yet on crates.io
//!    ```
//! 3. Enable the `usb-host` feature in this crate and uncomment the
//!    `use embassy_usb_host::*` line below.
//! 4. Implement `wait_for_device_event`, `bus_reset`, and `alloc_channel`,
//!    using the SYSCFG0.DCFM=1 hardware path in `super::regs`.
//! 5. Implement `UsbChannel::control_in/out`, `bulk_in/out`, `interrupt_in`.
//!    The FIFO routing for host mode is CFIFO→pipe0 for control, D0FIFO for
//!    ISO/bulk, D1FIFO for interrupt — identical to device mode but with
//!    DCFM=1 and the address field in PIPEMAXP.DEVSEL set to the device
//!    address.
//!
//! ## Hardware notes for host mode
//!
//! - Set SYSCFG0.DCFM=1 *before* USBE=1.
//! - Set DVSTCTR0.VBUSEN=1 to power VBUS.
//! - Wait for ATTCH (INTSTS1.ATTCH=1) to detect device connection.
//! - Drive SOF by setting DVSTCTR0.UACT=1.
//! - Full-speed only (DVSTCTR0.RHST=0b010 after reset).
//! - Host-mode interrupts use INTENB1: ATTCHE, DTCHE, BCHGE.
//! - DEVADD0-A registers hold per-device speed and hub info.

// TODO(host): uncomment once embassy PR #5633 merges and the dependency is
// added to Cargo.toml.
// use embassy_usb_host::{UsbHostDriver, UsbChannel, ChannelType, ChannelConfig};

use super::regs::{
    Rusb1Regs, SYSCFG_USBE, SYSCFG_DCFM, SYSCFG_DRPD,
    DVSTCTR0_VBUSEN, BUSWAIT_VALUE,
    rd, wr, rmw,
};

// ---------------------------------------------------------------------------
// Rusb1HostDriver (stub)
// ---------------------------------------------------------------------------

/// Host-mode driver for one RUSB1 port.
///
/// Currently a **non-functional stub**.  See module-level docs for the
/// implementation roadmap.
pub struct Rusb1HostDriver {
    port: u8,
}

impl Rusb1HostDriver {
    /// Create a host driver.  Puts the hardware into host mode (DCFM=1) and
    /// enables VBUS power (VBUSEN=1).
    ///
    /// # Safety
    /// Caller must ensure the module clock is running and no device driver is
    /// active on this port.
    pub unsafe fn new(port: u8) -> Self {
        let regs = Rusb1Regs::ptr(port);
        wr(core::ptr::addr_of_mut!((*regs).buswait), BUSWAIT_VALUE);
        // Host mode: DCFM=1
        rmw(core::ptr::addr_of_mut!((*regs).syscfg0),
            SYSCFG_DCFM | SYSCFG_DRPD, SYSCFG_DCFM);
        // Enable USB module.
        rmw(core::ptr::addr_of_mut!((*regs).syscfg0), SYSCFG_USBE, SYSCFG_USBE);
        // Power VBUS.
        rmw(core::ptr::addr_of_mut!((*regs).dvstctr0),
            DVSTCTR0_VBUSEN, DVSTCTR0_VBUSEN);
        Self { port }
    }

    /// Port number (0 or 1).
    pub fn port(&self) -> u8 { self.port }
}

// TODO(host): implement UsbHostDriver trait when embassy PR #5633 merges.
//
// impl UsbHostDriver for Rusb1HostDriver {
//     type Channel<T: ChannelType, D: embassy_usb_host::Dir> = Rusb1Channel<T, D>;
//
//     async fn wait_for_device_event(&mut self) -> DeviceEvent {
//         todo!("host: wait_for_device_event — see deluge-bsp/src/usb/host.rs")
//     }
//
//     async fn bus_reset(&mut self) {
//         todo!("host: bus_reset")
//     }
//
//     fn alloc_channel<T: ChannelType, D: embassy_usb_host::Dir>(
//         &mut self,
//         config: ChannelConfig,
//     ) -> Result<Self::Channel<T, D>, ChannelAllocError> {
//         todo!("host: alloc_channel")
//     }
// }

// ---------------------------------------------------------------------------
// Rusb1Channel (stub)
// ---------------------------------------------------------------------------

/// One bidirectional USB channel (host mode).
///
/// Currently a **non-functional stub**.
pub struct Rusb1Channel {
    port: u8,
    pipe: u8,
    ep_addr: u8,
}

// TODO(host): implement UsbChannel<T, D> for Rusb1Channel when PR #5633 merges.
//
// impl<T: ChannelType, D: embassy_usb_host::Dir> UsbChannel<T, D> for Rusb1Channel<T, D> {
//     async fn control_in(&mut self, setup: &SetupPacket, buf: &mut [u8])
//         -> Result<usize, ChannelError>
//     {
//         todo!("host: control_in")
//     }
//
//     async fn control_out(&mut self, setup: &SetupPacket, buf: &[u8])
//         -> Result<(), ChannelError>
//     {
//         todo!("host: control_out")
//     }
//
//     async fn bulk_in(&mut self, buf: &mut [u8]) -> Result<usize, ChannelError> {
//         todo!("host: bulk_in")
//     }
//
//     async fn bulk_out(&mut self, buf: &[u8]) -> Result<(), ChannelError> {
//         todo!("host: bulk_out")
//     }
//
//     async fn interrupt_in(&mut self, buf: &mut [u8]) -> Result<usize, ChannelError> {
//         todo!("host: interrupt_in")
//     }
// }
