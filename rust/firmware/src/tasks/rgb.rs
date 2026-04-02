use embassy_time::Timer;
use log::info;

use deluge_bsp::pic;
use crate::pads::{pad_get, pad_id_from_xy};

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
pub(crate) async fn rgb_task() {
    pic::wait_ready().await;

    let mut hue_offset: u8 = 0;
    // C: per-pair cache of the colours sent in the last frame.
    // Initialised to all-zeros so the first frame detects any lit pads as
    // changed.  Pairs that remain all-black are never re-sent.
    let mut last_sent: [[[u8; 3]; 16]; 9] = [[[0u8; 3]; 16]; 9];

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

            // Skip pairs whose output colours are identical to the last
            // transmission.  Saves the 49-byte DMA burst for dark pairs and
            // any pair the animation hasn't changed since the previous frame.
            if colours != last_sent[pair as usize] {
                pic::set_column_pair_rgb(pair, &colours).await;
                last_sent[pair as usize] = colours;
            }

            // Explicit yield between pairs so uac2_task can service any
            // USB packets that arrived while we were busy.  This is the
            // primary coop point when skipping the DMA await above.
            Timer::after_micros(0).await;
        }
        pic::done_sending_rows().await;

        hue_offset = hue_offset.wrapping_add(2); // full rainbow cycle every ~6.4 s
        Timer::after_millis(50).await;
    }
}
