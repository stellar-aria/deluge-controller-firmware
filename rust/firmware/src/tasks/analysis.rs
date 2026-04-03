//! Audio analysis task — FFT magnitude spectrum and waveform snapshot.
//!
//! Runs every 50 ms when the SSI RX DMA is active.  Publishes results to the
//! shared statics below for consumption by `oled_task` and `rgb_task`.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_time::Timer;
use rza1::ssi;

use deluge_fft::{Complex, RealFft, apply_hann_window_real};

// ---------------------------------------------------------------------------
// Shared output buffers
// ---------------------------------------------------------------------------

/// `true` while the SSI RX DMA write pointer is advancing.
pub(crate) static AUDIO_STREAMING: AtomicBool = AtomicBool::new(false);

/// Incremented each time a fresh analysis frame is published.
///
/// Consumers can detect new data by comparing against a cached epoch value.
pub(crate) static ANALYSIS_EPOCH: AtomicU32 = AtomicU32::new(0);

/// 128-point waveform snapshot, normalised to `[-1, 1]`.
///
/// Written only by `analysis_task`; read only by `oled_task`.
/// Safe in a single-threaded cooperative executor with no concurrent writers.
pub(crate) static mut WAVEFORM: [f32; 128] = [0.0; 128];

/// 257-bin magnitude spectrum (bins 0 … N/2 of a 512-point FFT), normalised
/// so that a full-scale sine wave produces a peak of ≈ 1.0.
///
/// Written only by `analysis_task`; read only by `rgb_task`.
pub(crate) static mut SPECTRUM: [f32; 257] = [0.0; 257];

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// FFT window length in left-channel samples (≈ 11.6 ms at 44.1 kHz).
const FFT_N: usize = 512;

/// SIMD lane width — 4 × f32 maps to a NEON `float32x4_t` on Cortex-A9.
const LANES: usize = 4;

/// Number of waveform points (equals `oled::WIDTH = 128`).
const WAVE_PTS: usize = 128;

// ---------------------------------------------------------------------------
// Task
// ---------------------------------------------------------------------------

/// Periodically analyses the SSI RX ring buffer.
///
/// Detection strategy: if `rx_current_ptr()` has not advanced since the
/// previous frame, the DMA is not running and we declare streaming inactive.
#[embassy_executor::task]
pub(crate) async fn analysis_task() {
    // Give hardware init and other tasks time to settle.
    Timer::after_millis(500).await;

    let base = ssi::rx_buf_start();
    let buf_len = ssi::RX_BUF_LEN; // 4096 interleaved i32 slots (stereo)

    let mut prev_ptr = ssi::rx_current_ptr();

    loop {
        Timer::after_millis(50).await;

        let cur_ptr = ssi::rx_current_ptr();
        let streaming = cur_ptr != prev_ptr;
        AUDIO_STREAMING.store(streaming, Ordering::Release);
        prev_ptr = cur_ptr;

        if !streaming {
            continue;
        }

        // ── Extract FFT_N left-channel samples ending at cur_ptr ────────────
        //
        // The RX ring buffer is interleaved stereo: slot 0=L, 1=R, 2=L, 3=R …
        // cur_ptr is the next slot the DMA will write (already committed data
        // is at cur_ptr-1 and earlier).
        //
        // We snap cur_ptr down to the nearest even (left-channel) index, then
        // walk back FFT_N × 2 mono slots to read FFT_N left samples.

        let cur_off = (cur_ptr as usize - base as usize) / core::mem::size_of::<i32>();
        // Round down to the most recent left-channel sample (even index).
        let head = cur_off & !1usize;
        // Start index (wraps inside the ring).
        let start = head.wrapping_sub(FFT_N * 2) % buf_len;

        // Stack-allocate the working buffers (~2 kB, down from ~4 kB).
        let mut wave_raw = [0.0f32; FFT_N];

        for (i, sample) in wave_raw.iter_mut().enumerate() {
            let idx = (start + i * 2) % buf_len;
            // SAFETY: idx is always within [0, buf_len).
            let raw: i32 = unsafe { base.add(idx).read_volatile() };
            // MSB-aligned 32-bit → f32 in [-1, 1].
            *sample = raw as f32 * (1.0 / 2_147_483_648.0_f32);
        }

        // ── Waveform snapshot (downsample FFT_N → WAVE_PTS) ─────────────────
        //
        // Pick every (FFT_N / WAVE_PTS) = 4th sample for a time-domain view.
        const STEP: usize = FFT_N / WAVE_PTS;
        unsafe {
            for i in 0..WAVE_PTS {
                WAVEFORM[i] = wave_raw[i * STEP];
            }
        }

        // ── FFT → magnitude spectrum ─────────────────────────────────────────
        // Apply Hann window to the real samples, then run a real-input FFT
        // (internally N/2-point radix-4 complex FFT + post-processing).
        // ~2× faster than a full N-point complex FFT.  Stack usage also halved.
        apply_hann_window_real(&mut wave_raw);
        let mut spec_cx = [Complex::ZERO; FFT_N / 2 + 1]; // 257 bins — halved stack
        RealFft::<FFT_N, LANES>::process(&wave_raw, &mut spec_cx);

        // Normalise: full-scale sine → RealFft peak ≈ N/4 (Hann halves amplitude).
        // Scale so peak → 1.0.
        let norm = 4.0 / FFT_N as f32;
        unsafe {
            for i in 0..257 {
                SPECTRUM[i] = spec_cx[i].abs() * norm;
            }
        }

        // Signal consumers that fresh data is available.
        ANALYSIS_EPOCH.fetch_add(1, Ordering::Release);
        deluge_bsp::oled::notify_redraw();
    }
}
