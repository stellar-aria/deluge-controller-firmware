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

        // Render snapshot and push.  send_frame awaits the PIC CS-echo (248),
        // which pic_task delivers by calling notify_oled_selected() above.
        render_pads(&mut fb);
        oled::send_frame(&fb).await;
    }
}
