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

use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use core::mem::MaybeUninit;
use log::{debug, error, info};

use embassy_executor::{Executor, Spawner};
use embassy_time::Timer;
use embassy_usb::driver::{Endpoint as _, EndpointIn as _, EndpointOut as _};

use deluge_bsp::cv_gate;
use deluge_bsp::oled;
use deluge_bsp::pic;
use deluge_bsp::sd;
use deluge_bsp::uart as bsp_uart;
use deluge_bsp::usb::{dcd_int_handler, Rusb1Driver};
use deluge_bsp::usb::classes::audio::{USB_BITS_PER_SAMPLE, USB_CAPTURE_BITS_PER_SAMPLE};
use rza1::{allocator, cache, gic, mmu, ostm, sdram, ssi, stb};

extern "C" {
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
// Shared pad state — atomic bit fields (no mutex, no allocation)
// ---------------------------------------------------------------------------
//
// 144 pads packed into 5 × u32 (160 bits; high 16 bits of word 4 unused).
// AtomicU32::fetch_xor provides lock-free single-bit toggle.

static PAD_BITS: [AtomicU32; 5] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
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
    (old >> (id % 32)) & 1 == 0 // was 0 (unlit) → now 1 (lit)
}

/// Set every pad to `val` (true = all lit, false = all dark).
fn pad_set_all(val: bool) {
    let fill = if val { !0u32 } else { 0u32 };
    for slot in PAD_BITS[..4].iter() {
        slot.store(fill, Ordering::Relaxed);
    }
    PAD_BITS[4].store(fill & 0x0000_FFFF, Ordering::Relaxed); // 16 valid bits
}

/// Flip every pad's lit state.
fn pad_invert_all() {
    for slot in PAD_BITS[..4].iter() {
        slot.fetch_xor(!0u32, Ordering::Relaxed);
    }
    PAD_BITS[4].fetch_xor(0x0000_FFFF, Ordering::Relaxed);
}

// ---------------------------------------------------------------------------
// OLED rendering
// ---------------------------------------------------------------------------
//
// 18 pad columns × 8 pad rows — each cell is CELL_W × CELL_H pixels.
// 18 × 7 = 126 px ≤ 128 (2 px right margin);  8 × 6 = 48 px exactly.

const CELL_W: usize = 7; // px per pad column  (18 × 7 = 126 ≤ 128)
const CELL_H: usize = 5; // px per pad row      ( 8 × 5 =  40 ≤ 43 visible)
const FILL_W: usize = 5; // CELL_W − 2px borders
const FILL_H: usize = 3; // CELL_H − 2px borders
/// First visible OLED row (rows 0–4 are off-panel, C constant OLED_MAIN_TOPMOST_PIXEL=5).
const TOPMOST: usize = 5;

/// Convert display grid position (x ∈ 0..18, y ∈ 0..8) to a pad ID 0–143.
///
/// `y` is in pad coordinates where y=0 is the **bottom** row (lower-left origin).
/// Inverse of [`pic::pad_coords`]:
/// - Even columns (`x % 2 == 0`): `id = y × 9 + x / 2`
/// - Odd  columns                : `id = (y + 8) × 9 + (x − 1) / 2`
#[inline]
fn pad_id_from_xy(x: u8, y: u8) -> u8 {
    if x.is_multiple_of(2) {
        y * 9 + x / 2
    } else {
        (y + 8) * 9 + (x - 1) / 2
    }
}

/// Render the current `PAD_BITS` state into `fb`, clearing first.
///
/// OLED rows 0–4 are off-panel (`TOPMOST`=5).  Pad y=0 is the physical
/// bottom row (lower-left origin), so `py=0` (OLED top cell) maps to pad
/// row y=7, and `py=7` (OLED bottom cell) maps to pad row y=0.
fn render_pads(fb: &mut oled::FrameBuffer) {
    fb.fill(0x00);
    for py in 0..8u8 {
        let pad_y = 7 - py; // flip: OLED top row = highest pad row
        for px in 0..18u8 {
            if !pad_get(pad_id_from_xy(px, pad_y)) {
                continue;
            }
            // 1 px gap on all sides → inner rect starts at (+1, +1)
            let ox = px as usize * CELL_W + 1;
            let oy = TOPMOST + py as usize * CELL_H + 1;
            for dy in 0..FILL_H {
                for dx in 0..FILL_W {
                    fb.set_pixel(ox + dx, oy + dy, true);
                }
            }
        }
    }
}

/// Map a 0–32 level to a 4-segment brightness bar.
///
/// Each 8-step increment fills one more segment.  Partial steps produce
/// intermediate brightness so movement is perceptible at any speed.
fn knob_brightnesses(level: i32) -> [u8; 4] {
    let mut b = [0u8; 4];
    for (i, slot) in b.iter_mut().enumerate() {
        let base = (i as i32) * 8;
        *slot = if level >= base + 8 {
            255
        } else if level > base {
            ((level - base) * 32) as u8 // 1‥7 → 32‥224
        } else {
            0
        };
    }
    b
}

/// Convert HSV to `[r, g, b]` (all 0–255).
///
/// `h`: hue 0–255 (mapped to 0–360°).  `s`: saturation 0–255.  `v`: value 0–255.
fn hsv_to_rgb(h: u8, s: u8, v: u8) -> [u8; 3] {
    if s == 0 {
        return [v, v, v];
    }
    let h6 = (h as u32 * 6) >> 8; // sector 0–5
    let f = ((h as u32 * 6) & 0xFF) as u8; // fractional part 0–255
    let p = ((v as u32 * (255 - s as u32)) >> 8) as u8;
    let q = ((v as u32 * (255 - ((s as u32 * f as u32) >> 8))) >> 8) as u8;
    let t = ((v as u32 * (255 - ((s as u32 * (255 - f as u32)) >> 8))) >> 8) as u8;
    match h6 {
        0 => [v, t, p],
        1 => [q, v, p],
        2 => [p, v, t],
        3 => [p, q, v],
        4 => [t, p, v],
        _ => [v, p, q],
    }
}

// ---------------------------------------------------------------------------
// Tasks
// ---------------------------------------------------------------------------

/// Heartbeat blink — P6_7 toggles every 500 ms so we can tell the CPU is alive.
#[embassy_executor::task]
async fn blink_task() {
    debug!("blink_task: started");
    let mut on = false;
    loop {
        on = !on;
        unsafe { rza1::gpio::write(6, 7, on) };
        Timer::after_millis(500).await;
    }
}

// ---------------------------------------------------------------------------
// USB audio shared state
// ---------------------------------------------------------------------------

/// Set to `true` by `uac2_task` while USB audio is streaming.
/// `jack_detect_task` reads this to gate `SPEAKER_ENABLE`.
static USB_STREAMING: AtomicBool = AtomicBool::new(false);

// Static buffers required by `embassy_usb::Builder`.
static mut USB_CONFIG_DESC: [u8; 512] = [0; 512];
static mut USB_BOS_DESC: [u8; 64] = [0; 64];
static mut USB_MSOS_DESC: [u8; 0] = [];
static mut USB_CONTROL_BUF: [u8; 64] = [0; 64];
/// Backing storage for the UAC2 `AudioClass` handler (must be `'static`).
static mut AUDIO_CLASS_BUF: core::mem::MaybeUninit<deluge_bsp::usb::classes::audio::AudioClass> =
    core::mem::MaybeUninit::uninit();

/// Small LFSR dither to prevent codec DC auto-mute.
///
/// The Akiyama codec auto-mutes after ~8192 consecutive identical samples
/// (≈0.19 s at 44.1 kHz).  We mix a ±16-LSB (of 24-bit) noise signal into
/// every sample written to the SSI TX buffer to keep the codec awake during
/// silence.  This matches the behaviour of the original C firmware.
#[inline]
fn dither_sample(lfsr: &mut u32) -> i32 {
    let bit = *lfsr & 1;
    *lfsr >>= 1;
    if bit != 0 {
        *lfsr ^= 0xB400;
    }
    // ±16 LSBs in the 24-bit range: noise in bits [12:8] of the 32-bit word.
    ((*lfsr & 0x1F) as i32 - 0x10) << 8
}

/// Fill the entire SSI TX buffer with low-level dither noise.
///
/// Called once at startup and after the USB stream ends to ensure the codec
/// never sees a burst of identical samples.
fn fill_tx_with_dither() {
    let buf_start = ssi::tx_buf_start();
    let buf_end = ssi::tx_buf_end();
    let mut lfsr: u32 = 0xACE1;
    let mut p = buf_start;
    while p < buf_end {
        unsafe {
            p.write_volatile(dither_sample(&mut lfsr));
            p = p.add(1);
        }
    }
}

/// Runs the `embassy_usb` device state machine (enumeration, control requests,
/// alternate-setting callbacks).  Must be kept alive at all times.
#[embassy_executor::task]
async fn usb_task(mut device: embassy_usb::UsbDevice<'static, Rusb1Driver>) {
    info!("usb_task: running");
    device.run().await;
}

/// UAC2 speaker task — reads isochronous OUT packets from the host and writes
/// them to the SSI TX buffer, keeping a fixed write-ahead lead ahead of the
/// DMA read head.
///
/// Packet format: stereo 24-bit LE PCM (3 bytes per mono sample).
/// SSI format: MSB-aligned 32-bit word (data in bits [31:8]).
///
/// Speaker-enable logic (matching the C firmware):
/// - `SPEAKER_ENABLE` (P4.1) is raised when the stream is active AND no
///   headphone or line-out jack is inserted.
/// - It is dropped when the stream stops (no data for >200 ms).
#[embassy_executor::task]
async fn uac2_task(mut ep_out: deluge_bsp::usb::Rusb1EndpointOut) {
    /// How far ahead of the DMA read head to keep the write pointer (mono i32 slots).
    /// 1024 slots = 512 stereo frames ≈ 11.6 ms at 44.1 kHz — a comfortable margin.
    const WRITE_AHEAD: usize = 1024;
    /// After this many consecutive empty/error reads we declare the stream stopped.
    const TIMEOUT_MS: u64 = 200;

    info!("uac2_task: waiting for USB enumeration");
    ep_out.wait_enabled().await;
    info!("uac2_task: ISO OUT endpoint enabled");

    let buf_start = ssi::tx_buf_start();
    let buf_end = ssi::tx_buf_end();
    let buf_len = ssi::TX_BUF_LEN;

    // Pre-fill with dither so the codec stays alive before streaming begins.
    fill_tx_with_dither();

    let mut write_ptr: *mut i32 = buf_start;
    let mut streaming = false;
    let mut last_rx = embassy_time::Instant::now();

    // 288 bytes: max ISO packet for stereo 24-bit at 48 kHz (48 frames × 6 B).
    let mut pkt: [u8; 288] = [0; 288];
    let mut lfsr: u32 = 0xACE1;

    // Diagnostics: track min/max packet size per second (8000 µSOFs).
    let mut diag_count: u32 = 0;
    let mut diag_min: usize = 288;
    let mut diag_max: usize = 0;

    loop {
        // Try to read one isochronous packet (non-blocking in the sense that
        // if the host sends nothing we get EndpointError::Disabled or a 0-byte
        // packet per the USB spec's zero-length handling).
        match ep_out.read(&mut pkt).await {
            Err(_) => {
                // Endpoint disabled — host stopped streaming or disconnected.
                if streaming {
                    info!("uac2_task: stream stopped (endpoint disabled)");
                    streaming = false;
                    USB_STREAMING.store(false, Ordering::Relaxed);
                    unsafe { rza1::gpio::write(4, 1, false) }; // SPEAKER_ENABLE off
                    fill_tx_with_dither();
                }
                // Wait for the endpoint to become active again.
                ep_out.wait_enabled().await;
                info!("uac2_task: ISO OUT endpoint re-enabled");
                last_rx = embassy_time::Instant::now();
                continue;
            }
            Ok(0) => {
                // Zero-length packet — host is pacing but sending no audio.
                // Check stream timeout.
                if streaming && last_rx.elapsed() > embassy_time::Duration::from_millis(TIMEOUT_MS)
                {
                    info!("uac2_task: stream timed out");
                    streaming = false;
                    USB_STREAMING.store(false, Ordering::Relaxed);
                    unsafe { rza1::gpio::write(4, 1, false) };
                    fill_tx_with_dither();
                }
                continue;
            }
            Ok(bytes_read) => {
                last_rx = embassy_time::Instant::now();

                // Packet size diagnostics — log once per ~8000 packets (≈1 s).
                if bytes_read > 0 {
                    if bytes_read < diag_min { diag_min = bytes_read; }
                    if bytes_read > diag_max { diag_max = bytes_read; }
                    diag_count += 1;
                    if diag_count >= 8_000 {
                        info!("uac2_task: pkt size min={} max={} (last 8k pkts)", diag_min, diag_max);
                        diag_count = 0;
                        diag_min = 288;
                        diag_max = 0;
                    }
                }

                let dma_ptr = ssi::tx_current_ptr();
                // CRSA can briefly read one-past-the-end during the DMA link-descriptor
                // reload at the buffer wrap boundary.  Wrap into [0, buf_len) so the
                // ahead calculation below doesn't see a spuriously small value.
                let dma_off = (unsafe { dma_ptr.offset_from(buf_start) } as usize) % buf_len;

                if !streaming {
                    // First data after silence — re-anchor the write pointer
                    // WRITE_AHEAD slots ahead of the DMA, snapped to a stereo
                    // frame boundary (even offset) to keep L/R channels correct.
                    let mut off = (dma_off + WRITE_AHEAD) % buf_len;
                    off &= !1; // snap to even (stereo frame boundary)
                    write_ptr = unsafe { buf_start.add(off) };
                    streaming = true;
                    USB_STREAMING.store(true, Ordering::Relaxed);

                    // Gate speaker: on if no headphone / line-out detected.
                    let hp = unsafe { rza1::gpio::read_pin(6, 5) };
                    let lol = unsafe { rza1::gpio::read_pin(6, 3) };
                    let lor = unsafe { rza1::gpio::read_pin(6, 4) };
                    unsafe { rza1::gpio::write(4, 1, !hp && !lol && !lor) };
                    info!("uac2_task: streaming started");
                }

        // ── Underrun guard ──────────────────────────────────────────────────
                // If the DMA has caught up to the write pointer, re-anchor.
                {
                    let wr_off = unsafe { write_ptr.offset_from(buf_start) } as usize;
                    let ahead = (wr_off + buf_len - dma_off) % buf_len;
                    if ahead < WRITE_AHEAD / 2 {
                        info!("uac2_task: underrun (ahead={} < {}), re-anchoring", ahead, WRITE_AHEAD / 2);
                        let mut off = (dma_off + WRITE_AHEAD) % buf_len;
                        off &= !1;
                        write_ptr = unsafe { buf_start.add(off) };
                    }
                }

                // ── Convert USB samples → MSB-aligned 32-bit SSI ───────────
                // Supports 16-bit (2 B/sample) and 24-bit (3 B/sample) LE PCM.
                // SSI expects audio in bits [31:8]; 24-bit shifts left by 8,
                // 16-bit shifts left by 16.  Dither prevents codec auto-mute.
                let bytes_per_sample = (USB_BITS_PER_SAMPLE.load(Ordering::Relaxed) / 8) as usize;
                let num_samples = bytes_read / bytes_per_sample;
                let src = &pkt[..bytes_read];
                for i in 0..num_samples {
                    let sample = if bytes_per_sample == 3 {
                        let b0 = src[i * 3] as u32;
                        let b1 = src[i * 3 + 1] as u32;
                        let b2 = src[i * 3 + 2] as u32;
                        (b0 << 8 | b1 << 16 | b2 << 24) as i32
                    } else {
                        // 16-bit signed LE → MSB-align in 32 bits
                        let v = (src[i * 2] as u16 | (src[i * 2 + 1] as u16) << 8) as i16;
                        (v as i32) << 16
                    } + dither_sample(&mut lfsr);
                    unsafe {
                        write_ptr.write_volatile(sample);
                        write_ptr = write_ptr.add(1);
                        if write_ptr >= buf_end {
                            write_ptr = buf_start;
                        }
                    }
                }
            }
        }
    }
}

/// UAC2 microphone capture task — reads from SSI RX and sends over ISO IN.
///
/// The ISO IN packet cadence provides **implicit feedback** for the speaker
/// stream: the host observes the IN packet rate and adapts how much data it
/// sends per SOF, correcting long-term clock drift without a separate feedback
/// endpoint.
#[embassy_executor::task]
async fn uac2_mic_task(mut ep_in: deluge_bsp::usb::Rusb1EndpointIn) {
    info!("uac2_mic_task: waiting for capture enable");
    ep_in.wait_enabled().await;
    info!("uac2_mic_task: capture enabled");

    let rx_start = ssi::rx_buf_start();
    let rx_len = ssi::RX_BUF_LEN;
    let mut read_ptr = ssi::rx_current_ptr();

    let mut pkt = [0u8; 288];

    loop {
        // Compute how many stereo frames the SSI RX DMA has captured since we
        // last sent.  This ties the implicit feedback signal directly to the
        // hardware AUDIO_X1 crystal clock rather than an assumed call rate,
        // so the host adapts its OUT rate to exactly match the SSI regardless
        // of async executor scheduling jitter.
        //
        // Over any long interval: total IN frames sent = total SSI frames
        // captured = 44 100 Hz.  Host converges to sending 44 100 frames/sec
        // OUT, eliminating the systematic rate mismatch that caused underruns.
        let bytes_per_sample = (USB_CAPTURE_BITS_PER_SAMPLE.load(Ordering::Relaxed) / 8) as usize;
        let max_frames = pkt.len() / (2 * bytes_per_sample);
        let rx_hw_off = unsafe {
            (ssi::rx_current_ptr().offset_from(rx_start) as usize) % rx_len
        };
        let read_off = unsafe { read_ptr.offset_from(rx_start) as usize };
        let captured_mono = (rx_hw_off + rx_len - read_off) % rx_len;
        // Integer divide by 2 for stereo frames; capped so we never exceed the
        // packet buffer.  The fractional remainder carries naturally into the
        // next call via the unchanged read_ptr, giving correct Bresenham-style
        // alternating 5/6 frame packets averaging exactly 44 100 Hz.
        let frames = (captured_mono / 2).min(max_frames);

        // Read `frames` stereo pairs from SSI RX; convert MSB-aligned i32 → USB PCM.
        // Format matches the active capture alt setting (16-bit or 24-bit LE).
        let nbytes = frames * 2 * bytes_per_sample;
        for i in 0..frames * 2 {
            let sample = unsafe { read_ptr.read_volatile() };
            // SSI audio is in bits [31:8]; shift right to get the significant bits.
            let off = i * bytes_per_sample;
            if bytes_per_sample == 3 {
                let val = (sample >> 8) as u32;
                pkt[off]     = (val & 0xFF) as u8;
                pkt[off + 1] = ((val >> 8) & 0xFF) as u8;
                pkt[off + 2] = ((val >> 16) & 0xFF) as u8;
            } else {
                // 16-bit: keep the top 16 bits of the MSB-aligned sample
                let val = (sample >> 16) as u16;
                pkt[off]     = (val & 0xFF) as u8;
                pkt[off + 1] = ((val >> 8) & 0xFF) as u8;
            }
            unsafe {
                read_ptr = read_ptr.add(1);
                if read_ptr >= rx_start.add(rx_len) {
                    read_ptr = rx_start;
                }
            }
        }

        match ep_in.write(&pkt[..nbytes]).await {
            Ok(()) => {}
            Err(_) => {
                info!("uac2_mic_task: capture stopped");
                ep_in.wait_enabled().await;
                info!("uac2_mic_task: capture re-enabled");
                // Re-anchor behind current DMA write position.
                read_ptr = ssi::rx_current_ptr();
            }
        }
    }
}

/// PIC32 event dispatcher — input handling and LED feedback.
///
/// | Event                  | Action                                              |
/// |------------------------|-----------------------------------------------------|
/// | `PadPress { id }`         | Toggle pad; signal OLED redraw; log (x, y)          |
/// | `ButtonPress BACK`        | Clear all pads + LEDs; signal redraw                |
/// | `ButtonPress FILL`        | Fill all pads + LEDs; signal redraw                 |
/// | `ButtonPress SELECT`      | Invert all pads; signal redraw                      |
/// | `ButtonPress other`       | Light indicator LED while held                      |
/// | `ButtonRelease`           | Extinguish indicator LED                            |
/// | `EncoderDelta { id:2/3 }` | Drive LowerGold / UpperGold indicator ring          |
/// | `EncoderDelta { other }`  | Logged to RTT                                       |
/// | `OledSelected`            | Forward to [`pic::notify_oled_selected()`]          |
/// | `OledDeselected`          | Forward to [`pic::notify_oled_deselected()`]        |
#[embassy_executor::task]
async fn pic_task() {
    info!("PIC: init (31 250 → 200 000 baud)");
    pic::init().await;
    info!("PIC: ready");

    let mut parser = pic::Parser::new();
    debug!("pic_task: entering main loop");

    loop {
        let byte = rza1::uart::read_byte(pic::UART_CH).await;
        let Some(event) = parser.push(byte) else {
            continue;
        };

        match event {
            // ---- Pad toggle ------------------------------------------------
            pic::Event::PadPress { id } => {
                let lit = pad_toggle(id);
                let (x, y) = pic::pad_coords(id);
                info!(
                    "pad {} ({},{}) → {}",
                    id,
                    x,
                    y,
                    if lit { "on" } else { "off" }
                );
                oled::notify_redraw();
            }
            pic::Event::PadRelease { .. } => {}

            // ---- Buttons ---------------------------------------------------
            pic::Event::ButtonPress { id } => {
                info!("button {} press", id);
                pic::led_on(id).await;
                match id {
                    pic::button::BACK => {
                        pad_set_all(false);
                        for led in 0..36u8 {
                            pic::led_off(led).await;
                        }
                        oled::notify_redraw();
                    }
                    pic::button::FILL => {
                        pad_set_all(true);
                        for led in 0..36u8 {
                            pic::led_on(led).await;
                        }
                        oled::notify_redraw();
                    }
                    pic::button::SELECT => {
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
            // Encoders are read from GPIO by encoder_task; PIC does not send
            // encoder bytes.  This arm is a fallback in case the PIC firmware
            // ever adds encoder reporting.
            pic::Event::EncoderDelta { id, delta } => {
                info!("encoder {} Δ{} (via PIC UART)", id, delta);
            }

            // ---- OLED CS handshake — must be forwarded --------------------
            pic::Event::OledSelected => {
                pic::notify_oled_selected();
            }
            pic::Event::OledDeselected => {
                pic::notify_oled_deselected();
            }

            // ---- Misc ------------------------------------------------------
            pic::Event::FirmwareVersion(v) => info!("PIC fw v{}", v),
            pic::Event::NoPresses => {}
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
    info!("SD: initialising");
    match sd::init().await {
        Ok(()) => {
            info!("SD: card ready (HC={})", sd::is_hc());
            let mut buf = [0u8; 512];
            match sd::read_sector(0, &mut buf).await {
                Ok(()) => {
                    info!(
                        "SD: sector 0 first 16 bytes: {:02x} {:02x} {:02x} {:02x} \
                         {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} \
                         {:02x} {:02x} {:02x} {:02x}",
                        buf[0],
                        buf[1],
                        buf[2],
                        buf[3],
                        buf[4],
                        buf[5],
                        buf[6],
                        buf[7],
                        buf[8],
                        buf[9],
                        buf[10],
                        buf[11],
                        buf[12],
                        buf[13],
                        buf[14],
                        buf[15],
                    );
                }
                Err(e) => error!("SD: read_sector(0) failed: {:?}", e),
            }
        }
        Err(e) => error!("SD: init failed: {:?}", e),
    }
}

/// RGB pad-grid demo task.
///
/// Renders the current pad state to the 18 × 8 RGB LED matrix every 50 ms.
///
/// - **Lit pads** glow with a per-column hue that slowly cycles over time,
///   making the whole grid rotate through the rainbow as pads are held.
/// - **Dark pads** are off (RGB 0 0 0).
///
/// The full grid is sent as 9 × `SET_COLOUR_FOR_TWO_COLUMNS` messages
/// followed by `DONE_SENDING_ROWS` (byte 240) to trigger the PIC's refresh.
#[embassy_executor::task]
async fn rgb_task() {
    pic::wait_ready().await;

    let mut hue_offset: u8 = 0;

    info!("rgb_task: started");

    loop {
        for pair in 0u8..9 {
            let col_l = pair * 2;
            let col_r = col_l + 1;
            let mut colours = [[0u8; 3]; 16];

            for row in 0..8u8 {
                // Left column: pad (col_l, row) — row 0 = bottom of both grid and PIC protocol
                if pad_get(pad_id_from_xy(col_l, row)) {
                    let hue = ((col_l as u16 * 256 / 18) as u8).wrapping_add(hue_offset);
                    colours[row as usize] = hsv_to_rgb(hue, 220, 180);
                }
                // Right column
                if pad_get(pad_id_from_xy(col_r, row)) {
                    let hue = ((col_r as u16 * 256 / 18) as u8).wrapping_add(hue_offset);
                    colours[8 + row as usize] = hsv_to_rgb(hue, 220, 180);
                }
            }

            pic::set_column_pair_rgb(pair, &colours).await;
        }
        pic::done_sending_rows().await;

        hue_offset = hue_offset.wrapping_add(2); // full rainbow cycle every ~6.4 s
        Timer::after_millis(50).await;
    }
}

/// Jack-detect and speaker-enable task.
///
/// Polls the five audio jack-detect GPIO inputs every 20 ms and logs any
/// changes.  Also drives the `SPEAKER_ENABLE` output: the speaker amplifier is
/// enabled only when no headphone or line-out plug is inserted.
///
/// | signal           | port/pin | direction |
/// |------------------|----------|-----------|
/// | HEADPHONE_DETECT | P6.5     | input     |
/// | LINE_IN_DETECT   | P6.6     | input     |
/// | MIC_DETECT       | P7.9     | input     |
/// | LINE_OUT_DETECT_L| P6.3     | input     |
/// | LINE_OUT_DETECT_R| P6.4     | input     |
/// | SPEAKER_ENABLE   | P4.1     | output    |
#[embassy_executor::task]
async fn jack_detect_task() {
    unsafe {
        rza1::gpio::set_as_input(6, 5); // HEADPHONE_DETECT
        rza1::gpio::set_as_input(6, 6); // LINE_IN_DETECT
        rza1::gpio::set_as_input(7, 9); // MIC_DETECT
        rza1::gpio::set_as_input(6, 3); // LINE_OUT_DETECT_L
        rza1::gpio::set_as_input(6, 4); // LINE_OUT_DETECT_R
        rza1::gpio::set_as_output(4, 1); // SPEAKER_ENABLE
        rza1::gpio::write(4, 1, false); // speaker off until we know the jack state
    }

    let mut prev_hp = false;
    let mut prev_li = false;
    let mut prev_mic = false;
    let mut prev_lol = false;
    let mut prev_lor = false;

    info!("jack_detect_task: started");

    loop {
        let hp = unsafe { rza1::gpio::read_pin(6, 5) };
        let li = unsafe { rza1::gpio::read_pin(6, 6) };
        let mic = unsafe { rza1::gpio::read_pin(7, 9) };
        let lol = unsafe { rza1::gpio::read_pin(6, 3) };
        let lor = unsafe { rza1::gpio::read_pin(6, 4) };

        if hp != prev_hp || li != prev_li || mic != prev_mic || lol != prev_lol || lor != prev_lor {
            prev_hp = hp;
            prev_li = li;
            prev_mic = mic;
            prev_lol = lol;
            prev_lor = lor;

            info!(
                "jack: hp={} line_in={} mic={} line_out_L={} line_out_R={}",
                hp, li, mic, lol, lor
            );

            // Speaker enable is gated on USB audio streaming being active.
            // We leave that to the audio task; do NOT enable the amplifier here
            // (powering it on with no signal produces audible noise).
        }

        Timer::after_millis(20).await;
    }
}

/// Quadrature encoder polling task.
///
/// All six rotary encoders are wired directly to CPU GPIO Port 1.
/// This task configures the 12 input
/// pins, then polls them every 200 µs and runs the same quadrature-decode
/// algorithm as the C firmware (`hardware_events.cpp`).
///
/// Gold knobs (encoder ids 2 = LowerGold, 3 = UpperGold) additionally drive
/// their indicator rings via [`pic::set_gold_knob_indicators`].
///
/// | id | name      | A-pin   | B-pin   |
/// |----|-----------|---------|---------|
/// |  0 | SCROLL_X  | P1.11   | P1.12   |
/// |  1 | TEMPO     | P1.7    | P1.6    |
/// |  2 | MOD_0     | P1.0    | P1.15   |
/// |  3 | MOD_1     | P1.5    | P1.4    |
/// |  4 | SCROLL_Y  | P1.8    | P1.10   |
/// |  5 | SELECT    | P1.2    | P1.3    |
#[embassy_executor::task]
async fn encoder_task() {
    // [port_a, pin_a, port_b, pin_b]
    const PINS: [(u8, u8, u8, u8); 6] = [
        (1, 11, 1, 12), // 0 SCROLL_X
        (1, 7, 1, 6),   // 1 TEMPO
        (1, 0, 1, 15),  // 2 MOD_0 (LowerGold)
        (1, 5, 1, 4),   // 3 MOD_1 (UpperGold)
        (1, 8, 1, 10),  // 4 SCROLL_Y
        (1, 2, 1, 3),   // 5 SELECT
    ];

    unsafe {
        for (pa, pina, pb, pinb) in PINS {
            rza1::gpio::set_as_input(pa, pina);
            rza1::gpio::set_as_input(pb, pinb);
        }
    }

    // Per-encoder state: (last_read_a, last_read_b, last_switch_a, last_switch_b,
    //                     last_change, enc_pos)
    // Initialise by reading actual pin state so we start in sync with hardware.
    let mut state: [(bool, bool, bool, bool, i8, i8); 6] = unsafe {
        core::array::from_fn(|i| {
            let (pa, pina, pb, pinb) = PINS[i];
            let a = rza1::gpio::read_pin(pa, pina);
            let b = rza1::gpio::read_pin(pb, pinb);
            (a, b, a, b, 0i8, 0i8)
        })
    };

    // Gold knob levels (encoder ids 2 and 3), range 0–32.
    let mut knob_level = [0i32; 2];

    info!("encoder_task: started (GPIO polling, 200 µs interval)");

    loop {
        for i in 0..6usize {
            let (pa, pina, pb, pinb) = PINS[i];
            let (lra, lrb, lsa, lsb, last_change, enc_pos) = &mut state[i];

            let a = unsafe { rza1::gpio::read_pin(pa, pina) };
            let b = unsafe { rza1::gpio::read_pin(pb, pinb) };

            // Only decode when both lines have changed from their last stable state.
            if a != *lsa && b != *lsb {
                let change: i8 = if *lra != *lsa {
                    // A already partially transitioned → A moved first
                    let c = if *lsa == *lsb { -1 } else { 1 };
                    *lsa = a;
                    c
                } else if *lrb != *lsb {
                    // B already partially transitioned → B moved first
                    let c = if *lsa == *lsb { 1 } else { -1 };
                    *lsb = b;
                    c
                } else {
                    // Both change simultaneously; use previous direction as tie-break
                    let c: i8 = if *last_change >= 0 { 2 } else { -2 };
                    *lsa = a;
                    *lsb = b;
                    c
                };

                *enc_pos += change;
                *last_change = change;

                let mut detents: i8 = 0;
                while *enc_pos > 2 {
                    *enc_pos -= 4;
                    detents += 1;
                }
                while *enc_pos < -2 {
                    *enc_pos += 4;
                    detents -= 1;
                }

                if detents != 0 {
                    if i == 2 || i == 3 {
                        let k = i - 2;
                        knob_level[k] = (knob_level[k] + detents as i32).clamp(0, 32);
                        let brightness = knob_brightnesses(knob_level[k]);
                        pic::set_gold_knob_indicators(k as u8, brightness).await;
                        info!("knob {} level {} → {:?}", k, knob_level[k], brightness);
                    } else {
                        info!("encoder {} Δ{}", i, detents);
                    }
                }
            }

            *lra = a;
            *lrb = b;
        }

        Timer::after_micros(200).await;
    }
}

/// OLED render task.
///
/// Initialises the SSD1309 then waits for [`oled::notify_redraw()`] on every
/// pad-state change before rendering and pushing a full 768-byte frame.
#[embassy_executor::task]
async fn oled_task() {
    // Wait for pic::init() to complete before issuing any PIC UART commands.
    // Both tasks start concurrently; without this barrier oled::init() would
    // race with the baud-rate handshake in pic::init().
    pic::wait_ready().await;

    info!("OLED: init");
    oled::init().await;
    info!("OLED: ready");

    let mut fb = oled::FrameBuffer::new();

    // Render the initial (empty) state immediately without waiting for a pad press.
    render_pads(&mut fb);
    oled::send_frame(&fb).await;

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
        let size = core::ptr::addr_of!(__sram_heap_end) as usize
            - start as usize;
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

    // Pre-fill SSI TX buffer with dither so the codec doesn't auto-mute before
    // the first USB stream arrives.
    fill_tx_with_dither();
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
    let (usb_device, ep_out, ep_in) = unsafe {
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

        (builder.build(), ep_out, ep_in)
    };
    info!("USB: UsbDevice built");

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
        spawner.spawn(blink_task().unwrap());
        spawner.spawn(usb_task(usb_device).unwrap());
        spawner.spawn(uac2_task(ep_out).unwrap());
        spawner.spawn(uac2_mic_task(ep_in).unwrap());
        spawner.spawn(pic_task().unwrap());
        spawner.spawn(encoder_task().unwrap());
        spawner.spawn(jack_detect_task().unwrap());
        spawner.spawn(rgb_task().unwrap());
        spawner.spawn(oled_task().unwrap());
        spawner.spawn(sd_task().unwrap());
    });
}
