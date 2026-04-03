//! Deluge interactive demo.
//!
//! ## What it does
//! - **Pad press** — toggles the pad lit/unlit; the OLED reflects the change instantly.
//! - **Pad release** — no-op (pads work as toggles, not momentary).
//! - **BACK button** — clears all pads + all indicator LEDs; LED lights while held.
//! - **FILL button** — fills all pads + all indicator LEDs; LED lights while held.
//! - **SELECT button** — inverts all pads; LED lights while held.
//! - **All other buttons** — indicator LED tracks the press/release; otherwise no-op.
//! - **Encoders** — gold knob (MOD_0/MOD_1) rotations drive their indicator rings.
//!   All other encoders are logged to RTT.
//! - **OLED** — live 18 × 8 miniature of the pad grid.
//!
//! ## OLED layout (128 × 48 px frame; rows 0–4 off-panel; 128 × 43 visible)
//! ```text
//! row 0..4 — off-panel (OLED_MAIN_TOPMOST_PIXEL = 5)
//! ┌────────────────────────────────────────────────┐  128 px wide
//! │  ████   ████   ████        ████          ████  │
//! │  ████   …18 columns × 7 px, 8 rows × 5 px…     │  43 px visible
//! └────────────────────────────────────────────────┘
//! ```
//! Origin: pad (0,0) = lower-left corner.  Each lit pad = filled 5 × 3 px rect
//! with a 1 px gap on all sides; rendering starts at pixel row `TOPMOST` (5).
//!
//! ## Task architecture
//! - **`pic_task`** — UART RX loop; updates `PAD_BITS` atomically, drives button LEDs,
//!   calls [`oled::notify_redraw()`], and forwards the OLED CS-echo signals.
//! - **`oled_task`** — calls [`oled::wait_redraw()`] then renders + sends a frame.
//!   While it awaits the PIC CS-echo, `pic_task` runs, reads the echo byte, and
//!   fires the [`pic::notify_oled_selected()`] signal that unblocks `oled_task`.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod events;
mod pads;
mod tasks;

use core::mem::MaybeUninit;
use core::panic::PanicInfo;
use log::{debug, error, info};

use embassy_executor::{Executor, Spawner};

use deluge_bsp::cv_gate;
use deluge_bsp::uart as bsp_uart;
use deluge_bsp::usb::{Rusb1Driver, dcd_int_handler};
use rza1::{allocator, cache, gic, mmu, ostm, sdram, stb};

unsafe extern "C" {
    /// Start of the free SRAM heap region (set by the linker script).
    static __sram_heap_start: u8;
    /// End of the free SRAM heap region (start of RTT/stack reservation).
    static __sram_heap_end: u8;
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("PANIC: {}", info);
    loop {
        core::hint::spin_loop();
    }
}

// ---------------------------------------------------------------------------
// USB descriptor static buffers
// ---------------------------------------------------------------------------
//
// Must live here (in the root crate) so the `'static` lifetimes required by
// `embassy_usb::Builder` and `builder.handler()` are satisfied.

static mut USB_CONFIG_DESC: [u8; 768] = [0; 768];
static mut USB_BOS_DESC: [u8; 64] = [0; 64];
static mut USB_MSOS_DESC: [u8; 0] = [];
static mut USB_CONTROL_BUF: [u8; 64] = [0; 64];
/// Backing storage for the UAC2 `AudioClass` handler (must be `'static`).
static mut AUDIO_CLASS_BUF: core::mem::MaybeUninit<deluge_bsp::usb::classes::audio::AudioClass> =
    core::mem::MaybeUninit::uninit();
/// Backing storage for the CDC-ACM class state (line-coding, control signals).
static mut CDC_ACM_STATE: embassy_usb::class::cdc_acm::State<'static> =
    embassy_usb::class::cdc_acm::State::new();

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

static mut EXECUTOR: MaybeUninit<Executor> = MaybeUninit::uninit();

#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    let channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 16384,
                name: "Terminal",
                section: ".rtt_buffer"   // ring buffer bytes in uncached RAM
            }
        }
        section_cb: ".rtt_buffer"        // _SEGGER_RTT control block in uncached RAM
    };
    rtt_target::set_print_channel(channels.up.0);
    rtt_target::init_logger();
    info!("Deluge demo firmware starting");
    info!("Pad paint demo: press pads to toggle, buttons 0/1/2 to clear/fill/invert");

    // Initialise the SRAM heap before any allocation from internal RAM.
    unsafe {
        let start = core::ptr::addr_of!(__sram_heap_start) as *mut u8;
        let size = core::ptr::addr_of!(__sram_heap_end) as usize - start as usize;
        allocator::SRAM.init(start, size);
    }
    info!("SRAM heap: initialised ({} KB)", {
        let s = core::ptr::addr_of!(__sram_heap_end) as usize
            - core::ptr::addr_of!(__sram_heap_start) as usize;
        s / 1024
    });

    unsafe { stb::init() };
    info!("STB: module clocks enabled");

    unsafe { mmu::init_and_enable() };
    info!("MMU: enabled");

    info!("cache: enabling L1...");
    unsafe { cache::l1_enable() };
    info!("cache: L1 enabled");

    info!("cache: enabling L2...");
    unsafe { cache::l2_init() };
    info!("cache: L2 enabled");

    info!("SDRAM: initialising...");
    unsafe { sdram::init() };
    info!("SDRAM: initialised");

    // Initialise the SDRAM heap now that the SDRAM window is accessible.
    unsafe { allocator::SDRAM.init(0x0C00_0000 as *mut u8, 64 * 1024 * 1024) };
    info!("SDRAM heap: initialised (64 MB)");

    info!("GIC: initialising...");
    unsafe { gic::init() };
    info!("GIC: initialised");

    info!("OSTM: enabling clock...");
    unsafe { ostm::enable_clock() };
    info!("OSTM: clock enabled");

    info!("time driver: initialising...");
    unsafe { rza1::time_driver::init() };
    info!("time driver: ready");

    info!("GPIO: configuring heartbeat LED...");
    unsafe { rza1::gpio::set_as_output(6, 7) };
    info!("GPIO: heartbeat LED P6_7 configured");

    info!("UART: initialising MIDI...");
    unsafe { bsp_uart::init_midi(31_250) };
    info!("UART: MIDI ready");

    info!("UART: initialising PIC...");
    unsafe { bsp_uart::init_pic(31_250) };
    info!("UART: SCIF0/1 @ 31 250 baud");

    info!("audio: initialising SSI0...");
    unsafe { deluge_bsp::audio::init() };
    info!("audio: SSI0 + DMA + codec running");

    // Pre-fill SSI TX buffer with dither so the codec does not auto-mute before
    // the first USB stream arrives.
    tasks::audio::fill_tx_with_dither();
    info!("audio: TX buffer pre-filled with dither");

    // RSPI0 init — shared between OLED (8-bit) and CV DAC (32-bit).
    // cv_gate::init() puts it in 32-bit mode; oled::init() will switch to 8-bit.
    unsafe { cv_gate::init() };
    info!("RSPI0: initialised via cv_gate::init");

    // ── USB Audio Class 2.0 device ──────────────────────────────────────────
    info!("USB: initialising USB0...");
    unsafe { rza1::rusb1::module_clock_enable(0) };

    // Register the USB0 ISR *before* IRQ is globally enabled.  The driver's
    // start() will call int_enable(0) when UsbDevice::run() begins.
    unsafe {
        gic::register(rza1::rusb1::USB0_IRQ, || dcd_int_handler(0));
    }

    // Build the UsbDevice using `'static` buffers so the device can be moved
    // into `usb_task` which requires `'static` arguments.
    let (usb_device, ep_out, ep_in, cdc, midi_sender, midi_receiver) = unsafe {
        let driver = Rusb1Driver::new(0);
        let mut config = embassy_usb::Config::new(0x16D0, 0x0EDA);
        config.manufacturer = Some("Synthstrom Audible");
        config.product = Some("Deluge");
        config.self_powered = false;
        config.max_power = 250; // 500 mA

        let mut builder = embassy_usb::Builder::new(
            driver,
            config,
            &mut *core::ptr::addr_of_mut!(USB_CONFIG_DESC),
            &mut *core::ptr::addr_of_mut!(USB_BOS_DESC),
            &mut *core::ptr::addr_of_mut!(USB_MSOS_DESC),
            &mut *core::ptr::addr_of_mut!(USB_CONTROL_BUF),
        );

        // Allocate the UAC2 speaker + mic interfaces (288 B covers stereo 24-bit @ 48 kHz).
        let (audio_instance, ep_out, ep_in) =
            deluge_bsp::usb::classes::audio::AudioClass::new(&mut builder, 288);
        // Store in a `'static` slot so the `&'static mut` reference satisfies
        // `builder.handler`'s `'d` lifetime (= `'static` here).
        let audio_ref = (&mut *core::ptr::addr_of_mut!(AUDIO_CLASS_BUF)).write(audio_instance);
        builder.handler(audio_ref);

        // CDC-ACM serial class (Deluge host protocol).
        let cdc = embassy_usb::class::cdc_acm::CdcAcmClass::new(
            &mut builder,
            &mut *core::ptr::addr_of_mut!(CDC_ACM_STATE),
            64,
        );

        // USB MIDI 1.0 class — 1 in-jack (DIN→USB), 1 out-jack (USB→DIN).
        let midi = embassy_usb::class::midi::MidiClass::new(&mut builder, 1, 1, 64);
        let (midi_sender, midi_receiver) = midi.split();

        (
            builder.build(),
            ep_out,
            ep_in,
            cdc,
            midi_sender,
            midi_receiver,
        )
    };
    info!("USB: UsbDevice built");

    info!("encoder: configuring interrupt-driven inputs...");
    unsafe { tasks::encoder::encoder_irq_init() };

    debug!("enabling IRQ...");
    unsafe { cortex_ar::interrupt::enable() };
    debug!("IRQ enabled OK");
    info!("IRQ: enabled — starting Embassy tasks");

    debug!("Executor::new()...");
    #[allow(static_mut_refs)]
    let executor = unsafe {
        EXECUTOR.write(Executor::new());
        EXECUTOR.assume_init_mut()
    };
    debug!("executor.run()...");
    executor.run(|spawner: Spawner| {
        spawner.spawn(tasks::blink::blink_task().unwrap());
        spawner.spawn(tasks::usb::usb_task(usb_device).unwrap());
        spawner.spawn(tasks::audio::uac2_task(ep_out).unwrap());
        spawner.spawn(tasks::audio::uac2_mic_task(ep_in).unwrap());
        spawner.spawn(tasks::pic::pic_task().unwrap());
        spawner.spawn(tasks::encoder::encoder_task().unwrap());
        spawner.spawn(tasks::jack_detect::jack_detect_task().unwrap());
        spawner.spawn(tasks::analysis::analysis_task().unwrap());
        spawner.spawn(tasks::rgb::rgb_task().unwrap());
        spawner.spawn(tasks::oled::oled_task().unwrap());
        spawner.spawn(tasks::sd::sd_task().unwrap());
        spawner.spawn(tasks::cdc::cdc_task(cdc).unwrap());
        spawner.spawn(tasks::midi::midi_usb_rx_task(midi_receiver).unwrap());
        spawner.spawn(tasks::midi::midi_din_tx_task(midi_sender).unwrap());
    });
}
