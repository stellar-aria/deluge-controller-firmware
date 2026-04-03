use embassy_time::Timer;
use log::info;

use deluge_bsp::pic;
use crate::pads::{pad_get, pad_id_from_xy};
use crate::tasks::analysis::{AUDIO_STREAMING, SPECTRUM};

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

/// Build the 9-pair colour array for the spectrum visualisation.
///
/// Maps 256 FFT magnitude bins (1 … 256, skipping DC) onto 18 frequency-band
/// columns.  Each column lights 0 – 8 rows from the bottom, VU-meter style.
/// The hue follows the column index so the grid shades from red (bass) to
/// violet (treble).
fn spectrum_colours() -> [[[u8; 3]; 16]; 9] {
    const N_COLS: usize = 18;
    const USABLE_BINS: usize = 256; // bins 1..256 (skip DC bin 0)

    // SAFETY: written only by analysis_task; no concurrent writer in a
    //         single-threaded cooperative executor.
    let spectrum = unsafe { &*core::ptr::addr_of!(SPECTRUM) };

    let mut pairs: [[[u8; 3]; 16]; 9] = [[[0u8; 3]; 16]; 9];

    for col in 0..N_COLS {
        // Linear bin mapping: divide the 256 usable bins evenly across 18 cols.
        let bin_lo = col * USABLE_BINS / N_COLS + 1; // +1 to skip DC
        let bin_hi = (col + 1) * USABLE_BINS / N_COLS + 1;

        // Average magnitude across this column's bin group.
        let mut avg = 0.0f32;
        for b in bin_lo..bin_hi {
            avg += spectrum[b];
        }
        avg /= (bin_hi - bin_lo) as f32;

        // Map [0, 1] → 0..8 lit rows (bottom-up, VU-meter style).
        let filled = (avg * 8.0).min(8.0) as usize;

        // Hue: bass columns are red (hue=0), treble columns are violet (hue≈200).
        let hue = (col as u16 * 200 / (N_COLS as u16 - 1)) as u8;
        let rgb = hsv_to_rgb(hue, 220, 180);

        let pair = col / 2;
        let side = col % 2;

        for row in 0..8usize {
            let color = if row < filled { rgb } else { [0u8; 3] };
            if side == 0 {
                pairs[pair][row] = color; // left column of the pair
            } else {
                pairs[pair][8 + row] = color; // right column of the pair
            }
        }
    }

    pairs
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
        if AUDIO_STREAMING.load(core::sync::atomic::Ordering::Acquire) {
            // ── Spectrum view ─────────────────────────────────────────────
            let all_colours = spectrum_colours();
            for pair in 0u8..9 {
                let colours = all_colours[pair as usize];
                if colours != last_sent[pair as usize] {
                    pic::set_column_pair_rgb(pair, &colours).await;
                    last_sent[pair as usize] = colours;
                }
                Timer::after_micros(0).await;
            }
        } else {
            // ── Pad glow view ─────────────────────────────────────────────
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
        }
        pic::done_sending_rows().await;

        hue_offset = hue_offset.wrapping_add(2); // full rainbow cycle every ~6.4 s
        Timer::after_millis(50).await;
    }
}
