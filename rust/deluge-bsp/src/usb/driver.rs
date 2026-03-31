//! RUSB1 USB device-mode driver — implements `embassy_usb_driver::Driver`.
//!
//! ## Architecture
//!
//! ```text
//! embassy-usb ──┐
//!               │  Driver<'a> trait
//!               ▼
//!          Rusb1Driver
//!         /          \
//!   Rusb1Bus    Rusb1ControlPipe
//!         \          /
//!        dcd_int_handler   ← GIC IRQ 73/74
//!              |
//!         pipe::{pipe_xfer_in_start, pipe_xfer_out_brdy, pipe_xfer_in_bemp}
//!              |
//!         fifo::{hw_to_sw_fifo, sw_to_hw_fifo}
//! ```
//!
//! ## Interrupt handling
//!
//! The ISR `dcd_int_handler` reads INTSTS0 / BRDYSTS / BEMPSTS each time
//! hardware fires the **GIC** interrupt and:
//!
//! - **VBINT** (VBUS change): queues a `BUS_EVENT_VBUS` bit.
//! - **DVST** (device-state change): queues reset / suspend / resume events.
//! - **CTRT** (control-transfer ready): records CTSQ and queues event.
//! - **BRDY** (buffer-ready for OUT pipe): calls `pipe_xfer_out_brdy`; wakes
//!   the per-pipe `AtomicWaker` on completion.
//! - **BEMP** (buffer-empty for IN pipe): calls `pipe_xfer_in_bemp`; wakes on
//!   completion.
//!
//! ## Register-interrupt caveat (RZ/A1L §28.3.8)
//!
//! `INTENB0`, `BEMPENB`, and `BRDYENB` must not be written before VBUS rises.
//! This driver enables them inside `Rusb1Bus::enable()` which is called after
//! `Driver::start()` once embassy-usb has confirmed VBUS.

use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicU16, AtomicU8, Ordering};
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::Timer;
use embassy_usb_driver::{
    Bus, ControlPipe, Direction, Driver, EndpointAddress, EndpointAllocError,
    EndpointError, EndpointInfo, EndpointOut, EndpointIn, EndpointType, Event,
    Unsupported,
};

use super::regs::{
    Rusb1Regs,
    SYSCFG_USBE, SYSCFG_UPLLE, SYSCFG_HSE, SYSCFG_DPRPU, SYSCFG_DRPD, SYSCFG_DCFM,
    BUSWAIT_VALUE, SUSPMODE_SUSPM,
    INTENB0_BRDYE, INTENB0_BEMPE, INTENB0_CTRE, INTENB0_DVSE, INTENB0_VBSE,
    INTSTS0_VBSTS, INTSTS0_VBINT, INTSTS0_DVST, INTSTS0_CTRT, INTSTS0_BRDY,
    INTSTS0_BEMP, INTSTS0_CTSQ_MASK, INTSTS0_DVSQ_MASK, INTSTS0_DVSQ_SHIFT,
    INTSTS0_VALID,
    DVSQ_DEFAULT, DVSQ_CONFIGURED, DVSQ_SUSP0,
    CTSQ_IDLE, CTSQ_READ_DATA, CTSQ_READ_STATUS, CTSQ_WRITE_DATA,
    CTSQ_WRITE_STATUS, CTSQ_WRITE_ZLP,
    PIPECTR_PID_NAK, PIPECTR_PID_BUF, PIPECTR_ACLRM, PIPECTR_CCPL,
    PIPECTR_SQCLR,
    DCPMAXP_MXPS_MASK,
    FIFOCTR_BCLR,
    PKT_BUF_BLOCKS, PKT_BUF_BLOCK_SIZE,
    rd, wr, rmw, pipectr_ptr,
};
use super::pipe::{
    BufAllocator, PipeConfig, XferType, PIPE_WAKERS, PIPE_DONE, PIPE_NRDY,
    PIPE_XFER, PIPE_COUNT, PipeXferState,
    pipe_configure, pipe_enable, pipe_disable, pipe_stall, pipe_reset,
    pipe_brdy_enable, pipe_brdy_disable, pipe_bemp_enable, pipe_bemp_disable,
    pipe_xfer_in_start, pipe_xfer_out_brdy, pipe_xfer_in_bemp,
};
use super::fifo::{FifoPort, fifo_for_pipe, fifo_select_pipe, fifo_is_ready,
                  sw_to_hw_fifo, hw_to_sw_fifo, fifo_dtln, fifo_bclr, fifo_bval};

// ---------------------------------------------------------------------------
// Global ISR-facing state
// ---------------------------------------------------------------------------

/// Bus-level events queued by the ISR and consumed by [`Rusb1Bus::poll`].
/// Bit layout (per port, index 0 = port 0):
///
/// - bit 0: VBUS rose
/// - bit 1: VBUS fell
/// - bit 2: USB bus reset
/// - bit 3: suspend
/// - bit 4: resume
static BUS_EVENTS: [AtomicU16; 2] = [AtomicU16::new(0), AtomicU16::new(0)];

/// Waker for [`Rusb1Bus::poll`] per port.
static BUS_WAKERS: [AtomicWaker; 2] = [AtomicWaker::new(), AtomicWaker::new()];

const BUS_EVT_VBUS_ON:  u16 = 1 << 0;
const BUS_EVT_VBUS_OFF: u16 = 1 << 1;
const BUS_EVT_RESET:    u16 = 1 << 2;
const BUS_EVT_SUSPEND:  u16 = 1 << 3;
const BUS_EVT_RESUME:   u16 = 1 << 4;

/// Pending CTSQ value delivered by the last CTRT interrupt (per port).
static CTRL_STAGE: [AtomicU8; 2] = [AtomicU8::new(0xFF), AtomicU8::new(0xFF)];

/// Waker for [`Rusb1ControlPipe::setup`] per port.
static CTRL_WAKERS: [AtomicWaker; 2] = [AtomicWaker::new(), AtomicWaker::new()];

/// Setup packet bytes captured from USBREQ/USBVAL/USBINDX/USBLENG per port.
static SETUP_PKT: [critical_section::Mutex<core::cell::UnsafeCell<[u8; 8]>>; 2] = {
    use core::cell::UnsafeCell;
    use critical_section::Mutex;
    const EMPTY: Mutex<UnsafeCell<[u8; 8]>> = Mutex::new(UnsafeCell::new([0u8; 8]));
    [EMPTY; 2]
};

// Sentinel meaning "no pending CTSQ".
const CTRL_STAGE_NONE: u8 = 0xFF;

// ---------------------------------------------------------------------------
// Per-port allocation bookkeeping (guarded by critical section)
// ---------------------------------------------------------------------------

struct PortAlloc {
    /// Which pipes are allocated: bit N = pipe N used (pipe 0 = DCP always used).
    pipes_used: u16,
    /// Endpoint → pipe mapping.  Index = ep_num, value = pipe number (0 = unused).
    ep_to_pipe: [[u8; 16]; 2], // [0]=OUT map, [1]=IN map
    /// Packet buffer block allocator.
    buf: BufAllocator,
    /// Pipe configuration (for re-configure on reset).
    pipe_cfg: [Option<PipeConfig>; PIPE_COUNT],
}

impl PortAlloc {
    const fn new() -> Self {
        Self {
            pipes_used: 0x0001, // pipe 0 pre-allocated
            ep_to_pipe: [[0u8; 16]; 2],
            buf: BufAllocator::new(),
            pipe_cfg: [None; PIPE_COUNT],
        }
    }

    fn alloc_pipe(&mut self, xfer_type: XferType, is_in: bool) -> Option<usize> {
        let (start, end) = match xfer_type {
            XferType::Control     => return Some(0),
            XferType::Isochronous => (1, 3),  // pipes 1-2
            XferType::Bulk        => (3, 10), // pipes 3-9
            XferType::Interrupt   => (6, 15), // pipes 6-14 (avoid ISO range)
        };
        for n in start..end {
            if self.pipes_used & (1 << n) == 0 {
                self.pipes_used |= 1 << n;
                return Some(n);
            }
        }
        None
    }
}

static PORT_ALLOC: [critical_section::Mutex<core::cell::UnsafeCell<PortAlloc>>; 2] = {
    use core::cell::UnsafeCell;
    use critical_section::Mutex;
    const INIT: Mutex<UnsafeCell<PortAlloc>> = Mutex::new(UnsafeCell::new(PortAlloc::new()));
    [INIT; 2]
};

// ---------------------------------------------------------------------------
// Rusb1Driver
// ---------------------------------------------------------------------------

/// The top-level USB device-mode driver for one RUSB1 port (0 or 1).
///
/// ## Safety invariant
/// Only one `Rusb1Driver` may exist per port at a time.  Enforced by the
/// `into_device_mode()` function in `mod.rs`.
pub struct Rusb1Driver {
    port: u8,
}

impl Rusb1Driver {
    /// Create a driver for `port` (0 or 1).
    ///
    /// # Safety
    /// Caller must ensure this is the only active driver for this port, and
    /// that `rza1::rusb1::module_clock_enable(port)` has already been called.
    pub unsafe fn new(port: u8) -> Self {
        debug_assert!(port <= 1);
        // Set BUSWAIT early so register accesses after USBE=1 are safe.
        let regs = Rusb1Regs::ptr(port);
        wr(core::ptr::addr_of_mut!((*regs).buswait), BUSWAIT_VALUE);
        Self { port }
    }
}

impl<'d> Driver<'d> for Rusb1Driver {
    type EndpointOut   = Rusb1EndpointOut;
    type EndpointIn    = Rusb1EndpointIn;
    type ControlPipe   = Rusb1ControlPipe;
    type Bus           = Rusb1Bus;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        _interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        let xfer_type = endpoint_type_to_xfer(ep_type);
        critical_section::with(|cs| {
            let alloc = unsafe { &mut *PORT_ALLOC[self.port as usize].borrow(cs).get() };
            let pipe = alloc.alloc_pipe(xfer_type, false).ok_or(EndpointAllocError)?;
            let buf_blocks = mps_to_blocks(max_packet_size);
            let buf_start = alloc.buf.alloc(buf_blocks).ok_or(EndpointAllocError)?;

            // Use requested ep address or pick a free one.
            let ep_num = if let Some(addr) = ep_addr {
                addr.index() as u8
            } else {
                (1u8..16)
                    .find(|&e| alloc.ep_to_pipe[0][e as usize] == 0)
                    .ok_or(EndpointAllocError)?
            };

            alloc.ep_to_pipe[0][ep_num as usize] = pipe as u8;
            alloc.pipe_cfg[pipe] = Some(PipeConfig {
                ep_num,
                is_in: false,
                xfer_type,
                mps: max_packet_size,
                buf_start,
                buf_blocks: buf_blocks as u8,
                double_buf: false,
            });

            let info = EndpointInfo {
                addr: EndpointAddress::from_parts(ep_num as usize, Direction::Out),
                ep_type,
                max_packet_size,
                interval_ms: _interval_ms,
            };
            Ok(Rusb1EndpointOut { port: self.port, pipe: pipe as u8, info })
        })
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        _interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        let xfer_type = endpoint_type_to_xfer(ep_type);
        critical_section::with(|cs| {
            let alloc = unsafe { &mut *PORT_ALLOC[self.port as usize].borrow(cs).get() };
            let pipe = alloc.alloc_pipe(xfer_type, true).ok_or(EndpointAllocError)?;
            let buf_blocks = mps_to_blocks(max_packet_size);
            let buf_start = alloc.buf.alloc(buf_blocks).ok_or(EndpointAllocError)?;

            let ep_num = if let Some(addr) = ep_addr {
                addr.index() as u8
            } else {
                (1u8..16)
                    .find(|&e| alloc.ep_to_pipe[1][e as usize] == 0)
                    .ok_or(EndpointAllocError)?
            };

            alloc.ep_to_pipe[1][ep_num as usize] = pipe as u8;
            alloc.pipe_cfg[pipe] = Some(PipeConfig {
                ep_num,
                is_in: true,
                xfer_type,
                mps: max_packet_size,
                buf_start,
                buf_blocks: buf_blocks as u8,
                double_buf: false,
            });

            let info = EndpointInfo {
                addr: EndpointAddress::from_parts(ep_num as usize, Direction::In),
                ep_type,
                max_packet_size,
                interval_ms: _interval_ms,
            };
            Ok(Rusb1EndpointIn { port: self.port, pipe: pipe as u8, info })
        })
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);

            // ── PLL / clock init (TRM §28.4.1 (5)) ─────────────────────────
            // Stop clock supply to this port while touching the PLL.
            rmw(core::ptr::addr_of_mut!((*regs).suspmode), SUSPMODE_SUSPM, 0);

            // UPLLE lives in USB0's SYSCFG0 regardless of which port we are
            // initialising (C reference: always writes to rusb0->SYSCFG0).
            let regs0 = Rusb1Regs::ptr(0);
            rmw(core::ptr::addr_of_mut!((*regs0).syscfg0), SYSCFG_UPLLE, SYSCFG_UPLLE);

            // Wait >= 1 ms for PLL lock (at ~400 MHz ≈ 400 K spin iterations).
            for _ in 0u32..400_000 {
                core::hint::spin_loop();
            }

            // CPU-bus wait cycles and clock re-enable.
            wr(core::ptr::addr_of_mut!((*regs).buswait), BUSWAIT_VALUE);
            rmw(core::ptr::addr_of_mut!((*regs).suspmode), SUSPMODE_SUSPM, SUSPMODE_SUSPM);

            // ── Mode / PHY config ────────────────────────────────────────────
            // HSE=1: module auto-negotiates HS or FS during the reset handshake.
            // Must be set while DPRPU=0 (TRM §28.3 SYSCFG.HSE).
            rmw(core::ptr::addr_of_mut!((*regs).syscfg0), SYSCFG_HSE, SYSCFG_HSE);

            // DCFM=0 (function/device mode), DRPD=0 (no pull-downs in device mode).
            rmw(core::ptr::addr_of_mut!((*regs).syscfg0), SYSCFG_DCFM | SYSCFG_DRPD, 0);

            // Enable USB module.
            rmw(core::ptr::addr_of_mut!((*regs).syscfg0), SYSCFG_USBE, SYSCFG_USBE);

            // D+ pull-up: notifies host of connection.
            rmw(core::ptr::addr_of_mut!((*regs).syscfg0), SYSCFG_DPRPU, SYSCFG_DPRPU);

            // DCPMAXP (control endpoint max packet size).
            wr(core::ptr::addr_of_mut!((*regs).dcpmaxp),
               control_max_packet_size & DCPMAXP_MXPS_MASK);

            // Enable VBUS interrupt so Bus::poll() can detect connection.
            wr(core::ptr::addr_of_mut!((*regs).intenb0), INTENB0_VBSE);

            // Register the ISR with the GIC.
            rza1::rusb1::int_enable(self.port);
        }
        (
            Rusb1Bus { port: self.port, connected: false },
            Rusb1ControlPipe { port: self.port, mps: control_max_packet_size },
        )
    }
}

// ---------------------------------------------------------------------------
// Rusb1Bus
// ---------------------------------------------------------------------------

pub struct Rusb1Bus {
    port: u8,
    /// Whether we currently believe VBUS is present.
    connected: bool,
}

impl Bus for Rusb1Bus {
    async fn poll(&mut self) -> Event {
        core::future::poll_fn(|cx| {
            BUS_WAKERS[self.port as usize].register(cx.waker());
            let ev = BUS_EVENTS[self.port as usize].swap(0, Ordering::Acquire);
            if ev & BUS_EVT_RESET != 0 {
                return Poll::Ready(Event::Reset);
            }
            if ev & BUS_EVT_SUSPEND != 0 {
                return Poll::Ready(Event::Suspend);
            }
            if ev & BUS_EVT_RESUME != 0 {
                return Poll::Ready(Event::Resume);
            }
            if ev & BUS_EVT_VBUS_ON != 0 {
                self.connected = true;
                return Poll::Ready(Event::PowerDetected);
            }
            if ev & BUS_EVT_VBUS_OFF != 0 {
                self.connected = false;
                return Poll::Ready(Event::PowerRemoved);
            }
            Poll::Pending
        }).await
    }

    async fn enable(&mut self) {
        // Enable the full interrupt set now that VBUS is confirmed.
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            let intenb = INTENB0_VBSE | INTENB0_DVSE | INTENB0_CTRE
                       | INTENB0_BRDYE | INTENB0_BEMPE;
            wr(core::ptr::addr_of_mut!((*regs).intenb0), intenb);

            // Configure allocated pipes on the hardware.
            configure_all_pipes(self.port);
        }
    }

    async fn disable(&mut self) {
        // TRM §28.3 (3): when disconnection from the USB host is recognised
        // (device mode), DPRPU and DCFM must be toggled in this exact sequence
        // before the bus can be reused (e.g. switching to host mode).
        //
        // Step 1: clear DPRPU — pull-up off; host sees SE0 (disconnect).
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            rmw(core::ptr::addr_of_mut!((*regs).syscfg0), SYSCFG_DPRPU, 0);
        }
        // Step 2: wait ≥ 1 µs.
        Timer::after_micros(1).await;
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            // Step 3: pulse DCFM to 1 (resets PHY data-line state machine).
            rmw(core::ptr::addr_of_mut!((*regs).syscfg0), SYSCFG_DCFM, SYSCFG_DCFM);
        }
        // Step 4: wait ≥ 200 ns (timer minimum is 1 µs — safe to over-wait).
        Timer::after_micros(1).await;
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            // Step 5: clear DCFM back to function (device) mode.
            rmw(core::ptr::addr_of_mut!((*regs).syscfg0), SYSCFG_DCFM, 0);
            // Disable all interrupts.
            wr(core::ptr::addr_of_mut!((*regs).intenb0), 0);
            wr(core::ptr::addr_of_mut!((*regs).brdyenb), 0);
            wr(core::ptr::addr_of_mut!((*regs).bempenb), 0);
        }
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        let pipe = ep_addr_to_pipe(self.port, ep_addr);
        if let Some(p) = pipe {
            unsafe {
                let regs = Rusb1Regs::ptr(self.port);
                if enabled {
                    pipe_enable(regs, p);
                    if ep_addr.direction() == Direction::In {
                        pipe_bemp_enable(regs, p);
                    } else {
                        pipe_brdy_enable(regs, p);
                    }
                } else {
                    pipe_disable(regs, p);
                    pipe_bemp_disable(regs, p);
                    pipe_brdy_disable(regs, p);
                }
            }
        }
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        let pipe = ep_addr_to_pipe(self.port, ep_addr);
        if let Some(p) = pipe {
            unsafe {
                let regs = Rusb1Regs::ptr(self.port);
                if stalled {
                    pipe_stall(regs, p);
                } else {
                    pipe_reset(regs, p);
                    pipe_enable(regs, p);
                }
            }
        }
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        let pipe = ep_addr_to_pipe(self.port, ep_addr);
        if let Some(p) = pipe {
            unsafe {
                let regs = Rusb1Regs::ptr(self.port);
                let ctr = rd(pipectr_ptr(regs, p));
                (ctr & super::regs::PIPECTR_PID_MASK) == super::regs::PIPECTR_PID_STALL
            }
        } else {
            false
        }
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

// ---------------------------------------------------------------------------
// Rusb1ControlPipe
// ---------------------------------------------------------------------------

pub struct Rusb1ControlPipe {
    port: u8,
    mps:  u16,
}

impl ControlPipe for Rusb1ControlPipe {
    fn max_packet_size(&self) -> usize {
        self.mps as usize
    }

    async fn setup(&mut self) -> [u8; 8] {
        core::future::poll_fn(|cx| {
            CTRL_WAKERS[self.port as usize].register(cx.waker());
            let stage = CTRL_STAGE[self.port as usize].swap(CTRL_STAGE_NONE, Ordering::Acquire);
            if stage != CTRL_STAGE_NONE && stage == CTSQ_READ_DATA as u8 {
                let pkt = critical_section::with(|cs| unsafe {
                    *SETUP_PKT[self.port as usize].borrow(cs).get()
                });
                return Poll::Ready(pkt);
            }
            Poll::Pending
        }).await
    }

    async fn data_out(
        &mut self,
        buf: &mut [u8],
        _first: bool,
        _last: bool,
    ) -> Result<usize, EndpointError> {
        // Read from CFIFO (pipe 0 OUT).
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            let fifo = FifoPort::cfifo(regs);
            fifo_select_pipe(&fifo, 0, false);
            let vld = fifo_dtln(&fifo) as usize;
            let len = buf.len().min(vld);
            if len > 0 {
                hw_to_sw_fifo(&fifo, buf.as_mut_ptr(), len);
            }
            fifo_bclr(&fifo);
            Ok(len)
        }
    }

    async fn data_in(
        &mut self,
        data: &[u8],
        _first: bool,
        last: bool,
    ) -> Result<(), EndpointError> {
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            let fifo = FifoPort::cfifo(regs);
            fifo_select_pipe(&fifo, 0, true); // ISEL=1 selects IN direction

            // Wait for the ISEL bit to take effect (CFIFOSEL hardware latency).
            // Matches C `process_pipe0_xfer` which spins up to 1000 iterations.
            let mut ready = false;
            for _ in 0..1000u32 {
                let sel = rd(core::ptr::addr_of!((*regs).cfifosel));
                if sel & super::regs::FIFOSEL_ISEL != 0 {
                    ready = true;
                    break;
                }
            }
            if !ready || !fifo_is_ready(&fifo, 0) {
                return Err(EndpointError::Disabled);
            }

            sw_to_hw_fifo(&fifo, data.as_ptr(), data.len());

            if last || data.len() < self.mps as usize {
                fifo_bval(&fifo);
            }

            // Wait for BEMP on pipe 0.
            core::future::poll_fn(|cx| {
                PIPE_WAKERS[0].register(cx.waker());
                let done = PIPE_DONE.fetch_and(!(1u16), Ordering::Acquire);
                if done & 1 != 0 {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            }).await;
        }
        Ok(())
    }

    async fn accept(&mut self) {
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            // Set CCPL (control completion) to ACK the status stage.
            let ctr = rd(pipectr_ptr(regs, 0));
            wr(pipectr_ptr(regs, 0), ctr | PIPECTR_CCPL);
        }
    }

    async fn reject(&mut self) {
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            // Set PID=STALL on DCP.  0b10 = STALL.
            wr(pipectr_ptr(regs, 0), super::regs::PIPECTR_PID_STALL);
        }
    }

    async fn accept_set_address(&mut self, addr: u8) {
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            // Hardware auto-updates USBADDR when CCPL is set.
            let _ = addr;
            let ctr = rd(pipectr_ptr(regs, 0));
            wr(pipectr_ptr(regs, 0), ctr | PIPECTR_CCPL);
        }
    }
}

// ---------------------------------------------------------------------------
// Rusb1EndpointOut
// ---------------------------------------------------------------------------

pub struct Rusb1EndpointOut {
    pub port: u8,
    pub pipe: u8,
    pub info: EndpointInfo,
}

impl embassy_usb_driver::Endpoint for Rusb1EndpointOut {
    fn info(&self) -> &EndpointInfo { &self.info }

    async fn wait_enabled(&mut self) {
        core::future::poll_fn(|cx| {
            PIPE_WAKERS[self.pipe as usize].register(cx.waker());
            unsafe {
                let regs = Rusb1Regs::ptr(self.port);
                let ctr = rd(pipectr_ptr(regs, self.pipe as usize));
                if (ctr & super::regs::PIPECTR_PID_MASK) == PIPECTR_PID_BUF {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            }
        }).await
    }
}

impl EndpointOut for Rusb1EndpointOut {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let pipe = self.pipe as usize;
        let mps  = self.info.max_packet_size;

        // Store the transfer state so the ISR can fill the buffer.
        critical_section::with(|cs| {
            let state = unsafe { &mut *PIPE_XFER[pipe].borrow(cs).get() };
            state.buf = unsafe { core::ptr::NonNull::new_unchecked(buf.as_mut_ptr()) };
            state.length = buf.len() as u16;
            state.remaining = buf.len() as u16;
            state.mps = mps;
            state.xfer_type = endpoint_type_to_xfer(self.info.ep_type);
        });

        // Enable BRDY for this pipe → ISR will call pipe_xfer_out_brdy.
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            pipe_brdy_enable(regs, pipe);
            pipe_enable(regs, pipe);
        }

        // Wait for ISR to complete the read.
        let nbytes = core::future::poll_fn(|cx| {
            PIPE_WAKERS[pipe].register(cx.waker());
            let done = PIPE_DONE.load(Ordering::Acquire);
            if done & (1u16 << pipe) != 0 {
                PIPE_DONE.fetch_and(!(1u16 << pipe), Ordering::Release);
                let state = critical_section::with(|cs| {
                    let s = unsafe { &*PIPE_XFER[pipe].borrow(cs).get() };
                    (s.length, s.remaining)
                });
                Poll::Ready(Ok((state.0 - state.1) as usize))
            } else if PIPE_NRDY.load(Ordering::Acquire) & (1u16 << pipe) != 0 {
                PIPE_NRDY.fetch_and(!(1u16 << pipe), Ordering::Release);
                Poll::Ready(Err(EndpointError::Disabled))
            } else {
                Poll::Pending
            }
        }).await;

        // Disable BRDY after transfer.
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            pipe_brdy_disable(regs, pipe);
        }

        nbytes
    }
}

// ---------------------------------------------------------------------------
// Rusb1EndpointIn
// ---------------------------------------------------------------------------

pub struct Rusb1EndpointIn {
    pub port: u8,
    pub pipe: u8,
    pub info: EndpointInfo,
}

impl embassy_usb_driver::Endpoint for Rusb1EndpointIn {
    fn info(&self) -> &EndpointInfo { &self.info }

    async fn wait_enabled(&mut self) {
        core::future::poll_fn(|cx| {
            PIPE_WAKERS[self.pipe as usize].register(cx.waker());
            unsafe {
                let regs = Rusb1Regs::ptr(self.port);
                let ctr = rd(pipectr_ptr(regs, self.pipe as usize));
                if (ctr & super::regs::PIPECTR_PID_MASK) == PIPECTR_PID_BUF {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            }
        }).await
    }
}

impl EndpointIn for Rusb1EndpointIn {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        let pipe = self.pipe as usize;
        let mps  = self.info.max_packet_size;

        critical_section::with(|cs| {
            let state = unsafe { &mut *PIPE_XFER[pipe].borrow(cs).get() };
            // Cast away const — we only read this pointer in the ISR.
            state.buf = unsafe { core::ptr::NonNull::new_unchecked(buf.as_ptr() as *mut u8) };
            state.length = buf.len() as u16;
            state.remaining = buf.len() as u16;
            state.mps = mps;
            state.xfer_type = endpoint_type_to_xfer(self.info.ep_type);
        });

        // Kick off the first packet; subsequent packets sent from BEMP ISR.
        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            pipe_bemp_enable(regs, pipe);
            pipe_enable(regs, pipe);
            pipe_xfer_in_start(regs, pipe, mps);
        }

        // Wait for all packets to drain.
        core::future::poll_fn(|cx| {
            PIPE_WAKERS[pipe].register(cx.waker());
            let done = PIPE_DONE.load(Ordering::Acquire);
            if done & (1u16 << pipe) != 0 {
                PIPE_DONE.fetch_and(!(1u16 << pipe), Ordering::Release);
                Poll::Ready(Ok(()))
            } else if PIPE_NRDY.load(Ordering::Acquire) & (1u16 << pipe) != 0 {
                PIPE_NRDY.fetch_and(!(1u16 << pipe), Ordering::Release);
                Poll::Ready(Err(EndpointError::Disabled))
            } else {
                Poll::Pending
            }
        }).await?;

        unsafe {
            let regs = Rusb1Regs::ptr(self.port);
            pipe_bemp_disable(regs, pipe);
        }

        Ok(())
    }
}

// ---------------------------------------------------------------------------
// ISR
// ---------------------------------------------------------------------------

/// USB device-mode interrupt handler.
///
/// Must be registered with the GIC for IRQ 73 (port 0) or 74 (port 1) before
/// `Rusb1Bus::enable()` enables the per-pipe interrupt bits.
///
/// # Safety
/// Must only be called from an interrupt context matching the port's GIC IRQ.
/// Single-core bare-metal only.
pub unsafe fn dcd_int_handler(port: u8) {
    let regs = Rusb1Regs::ptr(port);
    let p = port as usize;

    let sts = rd(core::ptr::addr_of!((*regs).intsts0));

    // Clear all active sticky flags atomically in a single write, preserving
    // the VALID bit (matches C ISR: don't write 0 to bits we didn't observe,
    // as hardware may have set new bits since the read).
    //
    // RC-W0 semantics: writing 0 to a bit clears it; writing 1 leaves it set.
    // So: ~(observed_flags) | VALID  →  clears observed bits, keeps VALID=1.
    const STICKY_FLAGS: u16 = INTSTS0_CTRT | INTSTS0_DVST | INTSTS0_VBINT;
    wr(core::ptr::addr_of_mut!((*regs).intsts0),
       !(STICKY_FLAGS & sts) | INTSTS0_VALID);

    // ── VBINT (VBUS change) ──────────────────────────────────────────────
    if sts & INTSTS0_VBINT != 0 {
        if sts & INTSTS0_VBSTS != 0 {
            BUS_EVENTS[p].fetch_or(BUS_EVT_VBUS_ON, Ordering::Release);
        } else {
            BUS_EVENTS[p].fetch_or(BUS_EVT_VBUS_OFF, Ordering::Release);
        }
        BUS_WAKERS[p].wake();
    }

    // ── DVST (device-state change) ───────────────────────────────────────
    if sts & INTSTS0_DVST != 0 {
        let dvsq = ((sts & INTSTS0_DVSQ_MASK) >> INTSTS0_DVSQ_SHIFT) as u16;
        let evt = match dvsq {
            d if d == DVSQ_DEFAULT    => BUS_EVT_RESET,
            d if d >= DVSQ_SUSP0     => BUS_EVT_SUSPEND,
            _                        => BUS_EVT_RESUME,
        };
        BUS_EVENTS[p].fetch_or(evt, Ordering::Release);
        BUS_WAKERS[p].wake();

        // On reset, run the proper bus-reset sequence then reconfigure pipes.
        if dvsq == DVSQ_DEFAULT {
            process_bus_reset(port);
        }
    }

    // ── CTRT (control transfer stage) ────────────────────────────────────
    if sts & INTSTS0_CTRT != 0 {
        // Re-read INTSTS0 to get the live VALID bit (matches C
        // `process_setup_packet` which re-reads before checking VALID).
        let intsts_live = rd(core::ptr::addr_of!((*regs).intsts0));
        if intsts_live & INTSTS0_VALID != 0 {
            // Clear CFIFO before latching the setup packet — any data left
            // over from the previous transfer must be discarded (matches C
            // `process_setup_packet`: rusb->CFIFOCTR = USB_CFIFOCTR_BCLR).
            wr(core::ptr::addr_of_mut!((*regs).cfifoctr), FIFOCTR_BCLR);

            // Capture setup packet from USBREQ/USBVAL/USBINDX/USBLENG.
            let req   = rd(core::ptr::addr_of!((*regs).usbreq));
            let val   = rd(core::ptr::addr_of!((*regs).usbval));
            let idx   = rd(core::ptr::addr_of!((*regs).usbindx));
            let len   = rd(core::ptr::addr_of!((*regs).usbleng));
            let pkt = [
                (req & 0xFF) as u8, (req >> 8) as u8,
                (val & 0xFF) as u8, (val >> 8) as u8,
                (idx & 0xFF) as u8, (idx >> 8) as u8,
                (len & 0xFF) as u8, (len >> 8) as u8,
            ];
            let cs = critical_section::CriticalSection::new();
            *SETUP_PKT[p].borrow(cs).get() = pkt;
        }

        let ctsq = (sts & INTSTS0_CTSQ_MASK) as u8;
        CTRL_STAGE[p].store(ctsq, Ordering::Release);
        CTRL_WAKERS[p].wake();
    }

    // ── BRDY (buffer ready — data received on OUT pipe) ──────────────────
    if sts & INTSTS0_BRDY != 0 {
        let brdysts = rd(core::ptr::addr_of!((*regs).brdysts));
        let brdyenb = rd(core::ptr::addr_of!((*regs).brdyenb));
        let active = brdysts & brdyenb;

        for n in 0..16usize {
            if active & (1 << n) == 0 { continue; }
            // Clear the BRDY flag for this pipe.
            wr(core::ptr::addr_of_mut!((*regs).brdysts), !((1u16) << n));

            if n == 0 {
                // Pipe 0 BRDY means setup/status data arrived — handled by CTRT.
                // Wake the control waker here too just in case.
                CTRL_WAKERS[p].wake();
            } else {
                let done = pipe_xfer_out_brdy(regs, n);
                if done {
                    PIPE_DONE.fetch_or(1u16 << n, Ordering::Release);
                    PIPE_WAKERS[n].wake();
                }
            }
        }
    }

    // ── BEMP (buffer empty — IN pipe sent its data) ───────────────────────
    if sts & INTSTS0_BEMP != 0 {
        let bempsts = rd(core::ptr::addr_of!((*regs).bempsts));
        let bempenb = rd(core::ptr::addr_of!((*regs).bempenb));
        let active  = bempsts & bempenb;

        for n in 0..16usize {
            if active & (1 << n) == 0 { continue; }
            wr(core::ptr::addr_of_mut!((*regs).bempsts), !((1u16) << n));

            if n == 0 {
                // Pipe 0 BEMP -> control status stage sent; wake control pipe.
                PIPE_DONE.fetch_or(1, Ordering::Release);
                PIPE_WAKERS[0].wake();
            } else {
                let done = pipe_xfer_in_bemp(regs, n);
                if done {
                    PIPE_DONE.fetch_or(1u16 << n, Ordering::Release);
                    PIPE_WAKERS[n].wake();
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn endpoint_type_to_xfer(t: EndpointType) -> XferType {
    match t {
        EndpointType::Control     => XferType::Control,
        EndpointType::Bulk        => XferType::Bulk,
        EndpointType::Interrupt   => XferType::Interrupt,
        EndpointType::Isochronous => XferType::Isochronous,
    }
}

/// Minimum 64-byte buffer blocks for a given MPS.
fn mps_to_blocks(mps: u16) -> usize {
    ((mps as usize) + PKT_BUF_BLOCK_SIZE - 1) / PKT_BUF_BLOCK_SIZE
}

/// Translate an `EndpointAddress` to a hardware pipe number for the given port.
fn ep_addr_to_pipe(port: u8, ep_addr: EndpointAddress) -> Option<usize> {
    let ep_num  = ep_addr.index();
    let dir_idx = if ep_addr.direction() == Direction::In { 1 } else { 0 };
    critical_section::with(|cs| {
        let alloc = unsafe { &*PORT_ALLOC[port as usize].borrow(cs).get() };
        let p = alloc.ep_to_pipe[dir_idx][ep_num] as usize;
        if p == 0 && ep_num != 0 { None } else { Some(p) }
    })
}

/// Perform the hardware bus-reset sequence, matching C `process_bus_reset`.
///
/// - Disables all pipe BRDY/BEMP except DCP pipe 0 BRDY.
/// - Clears and deselects all FIFO ports.
/// - Issues ACLRM + SQCLR on every active pipe.
/// - Aborts any in-progress transfers (wakes waiters with NRDY error).
/// - Then reconfigures all pipes so they are ready for the re-enumeration.
unsafe fn process_bus_reset(port: u8) {
    let regs = Rusb1Regs::ptr(port);
    let p = port as usize;

    // Only keep pipe 0 BRDY enabled — pipes 1-15 will be re-enabled when
    // dcd_edpt_open / iso_activate are called by the host stack.
    wr(core::ptr::addr_of_mut!((*regs).brdyenb), 0x0001);
    wr(core::ptr::addr_of_mut!((*regs).bempenb), 0x0000);

    // Clear the control FIFO.
    wr(core::ptr::addr_of_mut!((*regs).cfifoctr), FIFOCTR_BCLR);

    // Deselect D0FIFO and D1FIFO (CURPIPE = 0).
    wr(core::ptr::addr_of_mut!((*regs).d0fifosel), 0);
    wr(core::ptr::addr_of_mut!((*regs).d1fifosel), 0);

    // ACLRM + SQCLR every active pipe and abort any in-progress transfer.
    let mut abort_mask: u16 = 0;
    critical_section::with(|cs| {
        let alloc = &*PORT_ALLOC[p].borrow(cs).get();
        for n in 1..PIPE_COUNT {
            if alloc.pipe_cfg[n].is_some() {
                let ctr = pipectr_ptr(regs, n);
                // ACLRM clears the pipe FIFO and resets the toggle.
                wr(ctr, PIPECTR_ACLRM | PIPECTR_SQCLR);
                wr(ctr, 0);

                // Signal transfer failure to any waiting task.
                let state = &mut *PIPE_XFER[n].borrow(cs).get();
                if state.remaining != 0 {
                    state.remaining = 0;
                    state.buf = core::ptr::NonNull::dangling();
                    abort_mask |= 1u16 << n;
                }
            }
        }
    });

    // Wake aborted waiters (they will observe PIPE_NRDY and return an error).
    if abort_mask != 0 {
        PIPE_NRDY.fetch_or(abort_mask, Ordering::Release);
        for n in 1..16usize {
            if abort_mask & (1u16 << n) != 0 {
                PIPE_WAKERS[n].wake();
            }
        }
    }

    // Re-apply all pipe hardware configs so the host can re-enumerate.
    configure_all_pipes(port);
}

/// (Re-)configure all allocated pipes on the hardware.  Called on enable and
/// on bus reset.
unsafe fn configure_all_pipes(port: u8) {
    let regs = Rusb1Regs::ptr(port);
    critical_section::with(|cs| {
        let alloc = &*PORT_ALLOC[port as usize].borrow(cs).get();
        for (n, cfg_opt) in alloc.pipe_cfg.iter().enumerate() {
            if n == 0 { continue; } // DCP configured via DCPMAXP/DCPCFG
            if let Some(ref cfg) = cfg_opt {
                pipe_configure(regs, n, cfg);
            }
        }
    });
}
