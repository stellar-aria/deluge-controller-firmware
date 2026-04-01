//! Deluge OLED display driver (SSD1309, 128 × 48 pixels).
//!
//! ## Hardware
//! | Panel         | SSD1309 OLED controller                      |
//! |---------------|---------------------------------------------|
//! | Resolution    | 128 × 48 pixels                             |
//! | SPI channel   | RSPI0 (shared with CV DAC)                  |
//! | SPI clock     | 10 MHz, mode 0 (CPHA=0, CPOL=0)            |
//! | CS / DC / RST | Controlled via PIC32 UART (bytes 248–251)   |
//! | Frame size    | 6 pages × 128 columns = 768 bytes           |
//!
//! ## CS/DC pin control
//! The chip-select and data/command pins are **not** GPIO on the RZ/A1L side —
//! they are driven by the PIC32 co-processor in response to short UART commands
//! sent over SCIF1.  The handshake is:
//! 1. CPU sends `250` (DC_LOW) or `251` (DC_HIGH) to set the data/command mode.
//! 2. CPU sends `248` (SELECT_OLED); PIC asserts CS and echoes `248` back.
//! 3. CPU waits for the echo via [`pic::wait_oled_selected()`].
//! 4. CPU streams pixel or command bytes over RSPI0 (8-bit mode).
//! 5. CPU sends `249` (DESELECT_OLED); PIC de-asserts CS and echoes `249` back.
//!
//! ## Frame buffer layout
//! ```text
//! oledMainImage[page][column]
//!
//! page   0 → rows  0– 7   (top, but skipped on 48px display)
//! page   1 → rows  8–15   (skipped)
//! page   2 → rows 16–23   (first visible row on 48px panel)
//! ...
//! page   7 → rows 56–63
//!
//! Within each byte, bit 0 = top pixel of the 8-row group,
//!                  bit 7 = bottom pixel.
//! ```
//! The SSD1309 is configured with page address 2–7 in horizontal addressing
//! mode, so the 768-byte flat dump of pages[2..=7][0..128] fills the panel.
//!
//! ## Usage
//! ```rust,ignore
//! // Once at startup (inside Embassy task, after pic_task has set up the bus):
//! oled::init().await;
//!
//! // Every frame:
//! let mut fb = oled::FrameBuffer::new();
//! oled::set_pixel(&mut fb, 64, 24, true);
//! oled::send_frame(&fb).await;
//! ```

use rza1::rspi;

#[cfg(target_os = "none")]
use crate::pic;

// ── Dimensions ────────────────────────────────────────────────────────────────

/// Physical pixel columns.
pub const WIDTH:  usize = 128;
/// Physical pixel rows.
pub const HEIGHT: usize = 48;
/// Number of SSD1309 pages used (HEIGHT / 8).
pub const PAGES:  usize = HEIGHT / 8; // = 6
/// Total frame buffer size in bytes.
pub const FRAME_BYTES: usize = PAGES * WIDTH; // = 768

/// First SSD1309 page that maps to display row 0.
///
/// On a 128×64 controller driving a 48-line panel the top 16 lines (pages 0–1)
/// are off-screen.  Page addressing starts at `(64 - HEIGHT) / 8 = 2`.
const FIRST_PAGE: u8 = ((64 - HEIGHT) / 8) as u8; // = 2

/// RSPI channel shared between OLED and CV DAC.
const SPI_CH: u8 = 0;

// ── Frame buffer ──────────────────────────────────────────────────────────────

/// 128 × 48 monochrome frame buffer, organized as 6 pages of 128 bytes.
///
/// Index as `buf.pages[page][col]`.  Each byte packs 8 vertical pixels
/// (bit 0 = topmost row of the page, bit 7 = bottommost).
#[derive(Clone)]
pub struct FrameBuffer {
    pub pages: [[u8; WIDTH]; PAGES],
}

impl FrameBuffer {
    /// Create a blank (all pixels off) frame buffer.
    pub const fn new() -> Self {
        Self { pages: [[0u8; WIDTH]; PAGES] }
    }
}

impl Default for FrameBuffer {
    fn default() -> Self { Self::new() }
}

impl FrameBuffer {
    /// Set or clear the pixel at column `x` (0–127), row `y` (0–47).
    ///
    /// Silently ignores out-of-bounds coordinates.
    #[inline]
    pub fn set_pixel(&mut self, x: usize, y: usize, on: bool) {
        if x >= WIDTH || y >= HEIGHT { return; }
        let page = y / 8;
        let bit  = y % 8;
        if on {
            self.pages[page][x] |=  1 << bit;
        } else {
            self.pages[page][x] &= !(1 << bit);
        }
    }

    /// Return `true` if the pixel at (x, y) is set.
    #[inline]
    pub fn get_pixel(&self, x: usize, y: usize) -> bool {
        if x >= WIDTH || y >= HEIGHT { return false; }
        (self.pages[y / 8][x] >> (y % 8)) & 1 != 0
    }

    /// Fill all pixels with `byte` (0x00 = all off, 0xFF = all on).
    #[inline]
    pub fn fill(&mut self, byte: u8) {
        for page in self.pages.iter_mut() {
            page.fill(byte);
        }
    }

    /// XOR-invert all pixels.
    #[inline]
    pub fn invert(&mut self) {
        for page in self.pages.iter_mut() {
            for b in page.iter_mut() {
                *b ^= 0xFF;
            }
        }
    }

    /// Return a flat byte slice over all 768 frame bytes (page-major order).
    #[inline]
    pub fn as_bytes(&self) -> &[u8] {
        // Safety: [[u8; 128]; 6] has the same layout as [u8; 768].
        unsafe {
            core::slice::from_raw_parts(self.pages.as_ptr() as *const u8, FRAME_BYTES)
        }
    }
}

// ── SSD1309 init sequence helpers ─────────────────────────────────────────────

/// Complete SSD1309 initialisation command sequence, in order.
///
/// These are the commands used in `oledMainInit()` in the C firmware.
/// They configure the controller for 128×48 pixels with horizontal
/// page-addressing starting at page [`FIRST_PAGE`].
const INIT_CMDS: &[u8] = &[
    0xFD, 0x12,             // SET COMMAND LOCK — unlock (0x12 = allow all commands)
    0xAE,                   // DISPLAY OFF
    0x81, 0xFF,             // CONTRAST = maximum
    0xA4,                   // ENTIRE DISPLAY ON = off (normal RAM content)
    0xA6,                   // SET NORMAL DISPLAY (0xA7 = inverse)
    0x00, 0x10,             // SET COLUMN START ADDRESS (low nibble = 0, high nibble = 0)
    0x20, 0x00,             // SET MEMORY ADDRESSING MODE = horizontal
    0x22, FIRST_PAGE, 7,    // SET PAGE ADDRESS: start=FIRST_PAGE (2), end=7
    0x40,                   // SET DISPLAY START LINE = 0
    0xA0,                   // SET SEGMENT RE-MAP (A0 = normal, A1 = mirrored)
    0xA8, 0x3F,             // SET MULTIPLEX RATIO = 63 (64 COM lines on controller)
    0xC0,                   // COM OUTPUT SCAN DIRECTION: COM0→COM63 (top→bottom)
    0xD3, 0x00,             // SET DISPLAY OFFSET = 0
    0xDA, 0x12,             // COM PIN HARDWARE CONFIGURATION = sequential
    0xD5, 0xF0,             // SET DISPLAY CLOCK DIVIDE RATIO / OSCILLATOR FREQUENCY
    0xD9, 0xA2,             // SET PRE-CHARGE PERIOD
    0xDB, 0x34,             // SET VCOM DESELECT LEVEL
    0xAF,                   // DISPLAY ON
];

/// Send all INIT_CMDS bytes via RSPI0 (channel must already be in 8-bit mode).
///
/// # Safety
/// RSPI0 must be configured for 8-bit operation.
unsafe fn send_init_commands() {
    for &b in INIT_CMDS {
        rspi::send8(SPI_CH, b);
    }
    rspi::wait_end(SPI_CH);
}

// ── Public async API (embedded only) ─────────────────────────────────────────

/// Perform full OLED initialisation.
///
/// Mirrors the `setup_oled()` sequence from `oled_init.c`:
/// 1. Configure RSPI0 for 8-bit mode.
/// 2. Send PIC commands: DC_LOW, ENABLE_OLED, SELECT_OLED.
/// 3. Wait 5 ms for the PIC to power up the display.
/// 4. Send the 21-command SSD1309 init sequence.
/// 5. Switch DC to high (data mode for subsequent frame writes).
/// 6. Deselect via PIC.
///
/// **Must be called inside an Embassy task**, after `pic::init().await`
/// has completed.
#[cfg(target_os = "none")]
pub async fn init() {
    use embassy_time::Timer;
    log::debug!("oled: init");

    // ---- 1. Configure RSPI0 for 8-bit SPI mode ----------------------------
    unsafe { rspi::configure_8bit(SPI_CH) };

    // ---- 2. PIC: DC_LOW, ENABLE_OLED, SELECT_OLED -------------------------
    pic::oled_dc_low().await;
    pic::oled_enable().await;
    pic::oled_select().await;

    // ---- 3. Wait 5 ms (gives PIC time to assert CS and boost V_DD) --------
    Timer::after_millis(5).await;

    // ---- 4. Send SSD1309 init commands ------------------------------------
    unsafe { send_init_commands() };

    // ---- 5. Switch DC to high (data mode for frame writes) ----------------
    pic::oled_dc_high().await;

    // ---- 6. Deselect -------------------------------------------------------
    pic::oled_deselect().await;
    log::debug!("oled: ready");
}

/// Send a full 768-byte frame to the display.
///
/// The SPI CS handshake goes through the PIC:
/// 1. Send SELECT_OLED to PIC; wait for the echo (PIC asserts CS).
/// 2. Configure RSPI0 for 8-bit mode.
/// 3. Stream all 768 frame bytes via RSPI0 in a blocking loop (~614 µs at 10 MHz).
/// 4. Wait for TEND (shift register idle).
/// 5. Send DESELECT_OLED to PIC.
///
/// DC is assumed to already be high (data mode) from the init or the previous
/// frame transfer.
///
/// **Must be called inside an Embassy task.**
///
/// # Sharing RSPI0 with the CV DAC
/// This function reconfigures RSPI0 to 8-bit mode for the duration of the
/// frame transfer, then leaves it in 8-bit mode.  All CV DAC writes done via
/// [`crate::cv_gate::cv_set_blocking`] reconfigure RSPI0 to 32-bit mode
/// before writing and leave it there, so the two drivers take turns
/// naturally as long as they are not called concurrently.  Since Embassy is
/// single-threaded cooperative, this is safe as long as neither driver is
/// called from an interrupt context.
#[cfg(target_os = "none")]
pub async fn send_frame(fb: &FrameBuffer) {
    // Assert CS via PIC; wait for confirmation echo.
    pic::oled_select().await;
    pic::wait_oled_selected().await;

    // Stream the frame buffer.
    unsafe {
        rspi::configure_8bit(SPI_CH);
        for &byte in fb.as_bytes() {
            rspi::send8(SPI_CH, byte);
        }
        rspi::wait_end(SPI_CH);
    }

    // De-assert CS via PIC.
    pic::oled_deselect().await;
}

// ── Redraw signal (embedded only) ────────────────────────────────────────────
//
// Allow the firmware's UI task to notify the OLED render task that the display
// contents have changed without any shared-memory lock.

#[cfg(target_os = "none")]
mod redraw_signal {
    use core::future::poll_fn;
    use core::sync::atomic::{AtomicBool, Ordering};
    use core::task::Poll;
    use embassy_sync::waitqueue::AtomicWaker;

    static FLAG:  AtomicBool  = AtomicBool::new(true); // start dirty so first frame is sent
    static WAKER: AtomicWaker = AtomicWaker::new();

    pub fn notify() {
        FLAG.store(true, Ordering::Release);
        WAKER.wake();
    }

    pub async fn wait() {
        poll_fn(|cx| {
            WAKER.register(cx.waker());
            if FLAG.swap(false, Ordering::AcqRel) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

/// Mark the OLED display as needing a redraw.
///
/// Call this from any task whenever the contents to be displayed have changed.
/// [`wait_redraw()`] in the render task will unblock and push a new frame.
#[cfg(target_os = "none")]
#[inline]
pub fn notify_redraw() {
    redraw_signal::notify();
}

/// Suspend until [`notify_redraw()`] has been called since the last resume.
///
/// Suitable as the loop condition of a render task:
/// ```rust,ignore
/// loop {
///     oled::wait_redraw().await;
///     render_frame(&mut fb);
///     oled::send_frame(&fb).await;
/// }
/// ```
#[cfg(target_os = "none")]
#[inline]
pub async fn wait_redraw() {
    redraw_signal::wait().await;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn dimensions() {
        assert_eq!(WIDTH, 128);
        assert_eq!(HEIGHT, 48);
        assert_eq!(PAGES, 6);
        assert_eq!(FRAME_BYTES, 768);
        assert_eq!(FIRST_PAGE, 2);
    }

    #[test]
    fn framebuffer_zeroed_by_default() {
        let fb = FrameBuffer::new();
        assert!(fb.as_bytes().iter().all(|&b| b == 0));
    }

    #[test]
    fn framebuffer_fill() {
        let mut fb = FrameBuffer::new();
        fb.fill(0xFF);
        assert!(fb.as_bytes().iter().all(|&b| b == 0xFF));
    }

    #[test]
    fn set_and_get_pixel_top_left() {
        let mut fb = FrameBuffer::new();
        fb.set_pixel(0, 0, true);
        assert!(fb.get_pixel(0, 0));
        assert_eq!(fb.pages[0][0], 0b0000_0001); // bit 0 = row 0
    }

    #[test]
    fn set_and_get_pixel_bottom_right() {
        let mut fb = FrameBuffer::new();
        fb.set_pixel(127, 47, true);
        assert!(fb.get_pixel(127, 47));
        // row 47 → page 5 (47/8=5), bit 7 (47%8=7)
        assert_eq!(fb.pages[5][127], 0b1000_0000);
    }

    #[test]
    fn set_pixel_clears_correctly() {
        let mut fb = FrameBuffer::new();
        fb.fill(0xFF);
        fb.set_pixel(10, 10, false);
        // row 10 → page 1, bit 2
        assert_eq!(fb.pages[1][10], 0b1111_1011);
    }

    #[test]
    fn out_of_bounds_ignored() {
        let mut fb = FrameBuffer::new();
        fb.set_pixel(128, 0, true);  // column OOB
        fb.set_pixel(0, 48, true);   // row OOB
        assert!(!fb.get_pixel(128, 0));
        assert!(!fb.get_pixel(0, 48));
        assert!(fb.as_bytes().iter().all(|&b| b == 0));
    }

    #[test]
    fn invert_roundtrip() {
        let mut fb = FrameBuffer::new();
        fb.set_pixel(5, 5, true);
        fb.invert();
        assert!(!fb.get_pixel(5, 5));
        fb.invert();
        assert!(fb.get_pixel(5, 5));
    }

    #[test]
    fn as_bytes_length() {
        let fb = FrameBuffer::new();
        assert_eq!(fb.as_bytes().len(), FRAME_BYTES);
    }

    #[test]
    fn init_cmds_reasonable_length() {
        // SSD1309 init needs at least 10 command bytes; ours has 21.
        assert!(INIT_CMDS.len() >= 10);
    }

    #[test]
    fn page_addressing_for_48px() {
        // 128×48 display starts at page (64-48)/8 = 2
        assert_eq!(FIRST_PAGE, 2);
        // Verify INIT_CMDS contains the SET PAGE ADDRESS sequence
        let page_cmd_pos = INIT_CMDS.windows(3)
            .position(|w| w[0] == 0x22 && w[1] == FIRST_PAGE && w[2] == 7);
        assert!(page_cmd_pos.is_some(), "SET PAGE ADDRESS (0x22) not found in init commands");
    }
}
