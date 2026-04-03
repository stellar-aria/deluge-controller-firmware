use core::sync::atomic::{AtomicBool, Ordering};
use log::info;

use deluge_bsp::oled;
use deluge_bsp::pic;
use crate::pads::pad_get;
use crate::pads::pad_id_from_xy;

// ---------------------------------------------------------------------------
// OLED layout constants
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

// ---------------------------------------------------------------------------
// CDC-supplied framebuffer
// ---------------------------------------------------------------------------
//
// When a CDC host sends MSG_TO_UPDATE_DISPLAY, `set_cdc_display` stores the
// 768-byte frame here and sets CDC_FRAME_VALID.  The oled_task uses this
// framebuffer in preference to re-rendering from PAD_BITS.  On
// MSG_TO_CLEAR_DISPLAY or host disconnect, `clear_cdc_display` clears the
// flag and oled_task falls back to pad rendering.

static mut CDC_FRAME: [u8; oled::FRAME_BYTES] = [0u8; oled::FRAME_BYTES];
static CDC_FRAME_VALID: AtomicBool = AtomicBool::new(false);

/// Store a host-supplied OLED framebuffer and request a redraw.
///
/// `data` must be exactly [`oled::FRAME_BYTES`] bytes (768), laid out as
/// `data[page * oled::WIDTH + col]` (page-major, matching the SSD1309 wire
/// format).  Any extra bytes in `data` are ignored.
pub(crate) fn set_cdc_display(data: &[u8]) {
    let n = data.len().min(oled::FRAME_BYTES);
    // Safety: single-threaded cooperative executor; no concurrent writer.
    unsafe { CDC_FRAME[..n].copy_from_slice(&data[..n]); }
    CDC_FRAME_VALID.store(true, Ordering::Release);
    oled::notify_redraw();
}

/// Discard the host-supplied framebuffer and fall back to pad rendering.
pub(crate) fn clear_cdc_display() {
    CDC_FRAME_VALID.store(false, Ordering::Release);
    oled::notify_redraw();
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

/// OLED render task.
///
/// Initialises the SSD1309 then waits for [`oled::notify_redraw()`] on every
/// pad-state change before rendering and pushing a full 768-byte frame.
#[embassy_executor::task]
pub(crate) async fn oled_task() {
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

        if CDC_FRAME_VALID.load(Ordering::Acquire) {
            // Host supplied a framebuffer — copy directly and send.
            // Safety: CDC_FRAME is only mutated by set_cdc_display() which
            // runs from the CDC task; since the executor is single-threaded,
            // there is no concurrent access here.
            unsafe { fb.pages.as_flattened_mut().copy_from_slice(&*core::ptr::addr_of!(CDC_FRAME)); }
        } else {
            render_pads(&mut fb);
        }
        oled::send_frame(&fb).await;
    }
}
