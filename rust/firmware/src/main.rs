//! Deluge interactive demo.
//!
//! ## What it does
//! - **Pad press** — toggles the pad lit/unlit; the OLED reflects the change instantly.
//! - **Pad release** — no-op (pads work as toggles, not momentary).
//! - **Button 0 ("Clear")** — clears all pads + all indicator LEDs; LED 0 lights while held.
//! - **Button 1 ("Fill")**  — fills all pads + all indicator LEDs; LED 1 lights while held.
//! - **Button 2 ("Invert")** — inverts all pads; LED 2 lights while held.
//! - **Buttons 3–35** — indicator LED tracks the press/release; otherwise no-op.
//! - **Encoders** — logged to RTT.
//! - **OLED** — live 18 × 8 miniature of the pad grid.
//!
//! ## OLED layout (128 × 48 px)
//! ```text
//! ┌────────────────────────────────────────────────┐  128 px
//! │  ████   ████   ████        ████          ████   │
//! │  ████   ████   ████        ████          ████   │  row 0  (6 px)
//! │  ████   ████   ████        ████          ████   │
//! │  ████   ████   ████        ████          ████   │
//! │                                                  │  row 1  (empty)
//! │  …18 columns × 7 px, 8 rows × 6 px…             │
//! └────────────────────────────────────────────────┘  48 px
//! ```
//! Each lit pad = filled 5 × 4 px rect with a 1 px gap on all sides.
//! Unlit pads leave a black background.
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

use core::panic::PanicInfo;
use core::sync::atomic::{AtomicU32, Ordering};

use rtt_target::{rprintln, rtt_init_print};
use core::mem::MaybeUninit;

use embassy_executor::{Executor, Spawner};
use embassy_time::Timer;

use rza1::{gic, ostm, cache, mmu, sdram, ssi, stb};
use deluge_bsp::uart as bsp_uart;
use deluge_bsp::cv_gate;
use deluge_bsp::oled;
use deluge_bsp::pic;
use deluge_bsp::sd;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("PANIC: {}", info);
    loop { core::hint::spin_loop(); }
}

// ---------------------------------------------------------------------------
// Shared pad state — atomic bit fields (no mutex, no allocation)
// ---------------------------------------------------------------------------
//
// 144 pads packed into 5 × u32 (160 bits; high 16 bits of word 4 unused).
// AtomicU32::fetch_xor provides lock-free single-bit toggle.

static PAD_BITS: [AtomicU32; 5] = [
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0),
    AtomicU32::new(0), AtomicU32::new(0),
];

/// Return `true` if pad `id` (0–143) is currently lit.
#[inline]
fn pad_get(id: u8) -> bool {
    (PAD_BITS[id as usize / 32].load(Ordering::Relaxed) >> (id % 32)) & 1 != 0
}

/// Toggle pad `id`; return `true` if it is now lit.
#[inline]
fn pad_toggle(id: u8) -> bool {
    let old = PAD_BITS[id as usize / 32].fetch_xor(1 << (id % 32), Ordering::Relaxed);
    (old >> (id % 32)) & 1 == 0   // was 0 (unlit) → now 1 (lit)
}

/// Set every pad to `val` (true = all lit, false = all dark).
fn pad_set_all(val: bool) {
    let fill = if val { !0u32 } else { 0u32 };
    for i in 0..4 { PAD_BITS[i].store(fill,            Ordering::Relaxed); }
    PAD_BITS[4].store(fill & 0x0000_FFFF,               Ordering::Relaxed); // 16 valid bits
}

/// Flip every pad's lit state.
fn pad_invert_all() {
    for i in 0..4 { PAD_BITS[i].fetch_xor(!0u32,        Ordering::Relaxed); }
    PAD_BITS[4].fetch_xor(0x0000_FFFF,                  Ordering::Relaxed);
}

// ---------------------------------------------------------------------------
// OLED rendering
// ---------------------------------------------------------------------------
//
// 18 pad columns × 8 pad rows — each cell is CELL_W × CELL_H pixels.
// 18 × 7 = 126 px ≤ 128 (2 px right margin);  8 × 6 = 48 px exactly.

const CELL_W: usize = 7;   // px per pad column
const CELL_H: usize = 6;   // px per pad row
const FILL_W: usize = 5;   // CELL_W − 1px border on each side
const FILL_H: usize = 4;   // CELL_H − 1px border on each side

/// Convert display grid position (x ∈ 0..18, y ∈ 0..8) to a pad ID 0–143.
///
/// Inverse of [`pic::pad_coords`]:
/// - Even columns (`x % 2 == 0`): `id = y × 9 + x / 2`
/// - Odd  columns                : `id = (y + 8) × 9 + (x − 1) / 2`
#[inline]
fn pad_id_from_xy(x: u8, y: u8) -> u8 {
    if x % 2 == 0 {
        y * 9 + x / 2
    } else {
        (y + 8) * 9 + (x - 1) / 2
    }
}

/// Render the current `PAD_BITS` state into `fb`, clearing first.
fn render_pads(fb: &mut oled::FrameBuffer) {
    fb.fill(0x00);
    for py in 0..8u8 {
        for px in 0..18u8 {
            if !pad_get(pad_id_from_xy(px, py)) { continue; }
            // 1 px gap on all sides → inner rect starts at (+1, +1)
            let ox = px as usize * CELL_W + 1;
            let oy = py as usize * CELL_H + 1;
            for dy in 0..FILL_H {
                for dx in 0..FILL_W {
                    fb.set_pixel(ox + dx, oy + dy, true);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Tasks
// ---------------------------------------------------------------------------

/// Heartbeat blink — P6_7 toggles every 500 ms so we can tell the CPU is alive.
#[embassy_executor::task]
async fn blink_task() {
    let mut on = false;
    loop {
        on = !on;
        unsafe { rza1::gpio::write(6, 7, on) };
        Timer::after_millis(500).await;
    }
}

/// Continuous audio output — keeps the SSI0 DMA TX buffer filled with a
/// 1 kHz stereo square-wave test tone at ±25 % full scale.
#[embassy_executor::task]
async fn audio_task() {
    const HALF_PERIOD: usize = 22;            // 44 100 Hz / 1 000 Hz / 2
    const AMPLITUDE:   i32   = i32::MAX / 4;

    let buf_start = ssi::tx_buf_start();
    let buf_end   = ssi::tx_buf_end();

    rprintln!("audio: TX buffer {:p}..{:p}", buf_start, buf_end);
    Timer::after_millis(5).await;

    let mut write_ptr: *mut i32 = buf_start;
    let mut phase: usize = 0;

    loop {
        for _ in 0..256usize {
            let sample = if phase < HALF_PERIOD { AMPLITUDE } else { -AMPLITUDE };
            phase = (phase + 1) % (HALF_PERIOD * 2);
            unsafe {
                write_ptr.write_volatile(sample);
                write_ptr = write_ptr.add(1);
                if write_ptr >= buf_end { write_ptr = buf_start; }
                write_ptr.write_volatile(sample);
                write_ptr = write_ptr.add(1);
                if write_ptr >= buf_end { write_ptr = buf_start; }
            }
        }
        Timer::after_millis(3).await;
    }
}

/// PIC32 event dispatcher — input handling and LED feedback.
///
/// | Event                  | Action                                              |
/// |------------------------|-----------------------------------------------------|
/// | `PadPress { id }`      | Toggle pad; signal OLED redraw; log (x, y)          |
/// | `ButtonPress 0`        | Clear all pads + LEDs 0–35; signal redraw           |
/// | `ButtonPress 1`        | Fill all pads + LEDs 0–35; signal redraw            |
/// | `ButtonPress 2`        | Invert all pads; signal redraw                      |
/// | `ButtonPress 3–35`     | Light indicator LED                                 |
/// | `ButtonRelease 0–35`   | Extinguish indicator LED                            |
/// | `OledSelected`         | Forward to [`pic::notify_oled_selected()`]          |
/// | `OledDeselected`       | Forward to [`pic::notify_oled_deselected()`]        |
#[embassy_executor::task]
async fn pic_task() {
    rprintln!("PIC: init (31 250 → 200 000 baud)");
    pic::init().await;
    rprintln!("PIC: ready");

    let mut parser = pic::Parser::new();

    loop {
        let byte = rza1::uart::read_byte(pic::UART_CH).await;
        let Some(event) = parser.push(byte) else { continue };

        match event {
            // ---- Pad toggle ------------------------------------------------
            pic::Event::PadPress { id } => {
                let lit = pad_toggle(id);
                let (x, y) = pic::pad_coords(id);
                rprintln!("pad {} ({},{}) → {}", id, x, y, if lit { "on" } else { "off" });
                oled::notify_redraw();
            }
            pic::Event::PadRelease { .. } => {}

            // ---- Buttons ---------------------------------------------------
            pic::Event::ButtonPress { id } => {
                rprintln!("button {} press", id);
                pic::led_on(id).await;
                match id {
                    0 => {
                        pad_set_all(false);
                        for led in 0..36u8 { pic::led_off(led).await; }
                        oled::notify_redraw();
                    }
                    1 => {
                        pad_set_all(true);
                        for led in 0..36u8 { pic::led_on(led).await; }
                        oled::notify_redraw();
                    }
                    2 => {
                        pad_invert_all();
                        oled::notify_redraw();
                    }
                    _ => {}
                }
            }
            pic::Event::ButtonRelease { id } => {
                pic::led_off(id).await;
            }

            // ---- Encoders --------------------------------------------------
            pic::Event::EncoderDelta { id, delta } => {
                rprintln!("encoder {} Δ{}", id, delta);
            }

            // ---- OLED CS handshake — must be forwarded --------------------
            pic::Event::OledSelected   => pic::notify_oled_selected(),
            pic::Event::OledDeselected => pic::notify_oled_deselected(),

            // ---- Misc ------------------------------------------------------
            pic::Event::FirmwareVersion(v) => rprintln!("PIC fw v{}", v),
            pic::Event::NoPresses          => {}
        }
    }
}

/// SD card probe task.
///
/// Initialises the SD controller and reads the first sector (the partition
/// table / boot block).  Logs the first 16 bytes to RTT so you can verify
/// the card is being read correctly.
#[embassy_executor::task]
async fn sd_task() {
    rprintln!("SD: initialising");
    match sd::init().await {
        Ok(()) => {
            rprintln!("SD: card ready (HC={})", sd::is_ready());
            let mut buf = [0u8; 512];
            match sd::read_sector(0, &mut buf).await {
                Ok(()) => {
                    rprintln!(
                        "SD: sector 0 first 16 bytes: {:02x} {:02x} {:02x} {:02x} \
                         {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} \
                         {:02x} {:02x} {:02x} {:02x}",
                        buf[0], buf[1], buf[2],  buf[3],
                        buf[4], buf[5], buf[6],  buf[7],
                        buf[8], buf[9], buf[10], buf[11],
                        buf[12], buf[13], buf[14], buf[15],
                    );
                }
                Err(e) => rprintln!("SD: read_sector(0) failed: {:?}", e),
            }
        }
        Err(e) => rprintln!("SD: init failed: {:?}", e),
    }
}

/// OLED render task.
///
/// Initialises the SSD1309 then waits for [`oled::notify_redraw()`] on every
/// pad-state change before rendering and pushing a full 768-byte frame.
#[embassy_executor::task]
async fn oled_task() {
    rprintln!("OLED: init");
    oled::init().await;
    rprintln!("OLED: ready");

    let mut fb = oled::FrameBuffer::new();

    loop {
        // Suspend until pic_task (or any other caller) marks the state dirty.
        oled::wait_redraw().await;

        // Render snapshot and push.  send_frame awaits the PIC CS-echo (248),
        // which pic_task delivers by calling notify_oled_selected() above.
        render_pads(&mut fb);
        oled::send_frame(&fb).await;
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

static mut EXECUTOR: MaybeUninit<Executor> = MaybeUninit::uninit();

#[no_mangle]
pub extern "C" fn main() -> ! {
    rtt_init_print!();
    rprintln!("Deluge demo firmware starting");
    rprintln!("Pad paint demo: press pads to toggle, buttons 0/1/2 to clear/fill/invert");

    unsafe { stb::init() };
    rprintln!("STB: module clocks enabled");

    unsafe { mmu::init_and_enable() };
    rprintln!("MMU: enabled");

    unsafe { cache::l1_enable() };
    unsafe { cache::l2_init() };
    rprintln!("caches: L1 + L2 enabled");

    unsafe { sdram::init() };
    rprintln!("SDRAM: initialised");

    unsafe { gic::init() };
    rprintln!("GIC: initialised");

    unsafe {
        ostm::enable_clock();
        rza1::time_driver::init();
    }
    rprintln!("time driver: ready");

    unsafe { rza1::gpio::set_as_output(6, 7) };   // heartbeat LED P6_7

    unsafe {
        bsp_uart::init_midi(31_250);   // SCIF0 — kept idle in this demo
        bsp_uart::init_pic(31_250);    // SCIF1 — PIC32
    }
    rprintln!("UART: SCIF0/1 @ 31 250 baud");

    unsafe { deluge_bsp::audio::init() };
    rprintln!("audio: SSI0 + DMA + codec running");

    // RSPI0 init — shared between OLED (8-bit) and CV DAC (32-bit).
    // cv_gate::init() puts it in 32-bit mode; oled::init() will switch to 8-bit.
    unsafe { cv_gate::init() };
    rprintln!("RSPI0: initialised via cv_gate::init");

    unsafe { cortex_ar::interrupt::enable() };
    rprintln!("IRQ: enabled — starting Embassy tasks");

    #[allow(static_mut_refs)]
    let executor = unsafe {
        EXECUTOR.write(Executor::new());
        EXECUTOR.assume_init_mut()
    };
    executor.run(|spawner: Spawner| {
        spawner.spawn(blink_task().unwrap());
        spawner.spawn(audio_task().unwrap());
        spawner.spawn(pic_task().unwrap());
        spawner.spawn(oled_task().unwrap());
        spawner.spawn(sd_task().unwrap());
    });
}
