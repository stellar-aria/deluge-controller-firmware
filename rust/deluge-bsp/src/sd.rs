//! Deluge-BSP SD card driver.
//!
//! Exposes an async interface for reading and writing 512-byte sectors on the
//! Deluge's SD card (SDHI port 1).
//!
//! ## SD protocol overview
//!
//! [`init`] runs the full SD v2 initialization sequence:
//!   CMD0  → reset card to idle
//!   CMD8  → check host voltage (determines SDHC/SDXC support)
//!   ACMD41 (loop) → wait for card ready, learn high-capacity flag
//!   CMD2  → get CID (ignored here)
//!   CMD3  → get RCA
//!   CMD7  → select card
//!   ACMD6 → switch to 4-bit bus
//!   CMD16 → set block length to 512 (needed for SDSC cards)
//!   Switch clock from 260 kHz to ~33 MHz
//!
//! Sector addressing:
//!   - SDHC/SDXC cards: block address (LBA directly)
//!   - SDSC cards: byte address (LBA × 512)
//!
//! ## Usage
//!
//! ```ignore
//! sd::init().await.expect("SD init failed");
//! let mut buf = [0u8; 512];
//! sd::read_sector(0, &mut buf).await.expect("read failed");
//! ```

use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};

use rza1::sdhi::{self, SdhiError};

// The Deluge SD card is on SDHI port 1.
const SD_PORT: u8 = 1;

// ---------------------------------------------------------------------------
// Well-known SD command register values (written to SD_CMD register).
//
// Most commands: lower 6 bits = command index.  The SDHI hardware infers the
// response type from the command index internally.
// CMD8 has bits [10:8] set (= 0x0408) to select the R7 response path.
// ---------------------------------------------------------------------------

const CMD0:  u16 = 0;   // GO_IDLE_STATE — no response
const CMD2:  u16 = 2;   // ALL_SEND_CID — R2
const CMD3:  u16 = 3;   // SEND_RELATIVE_ADDR — R6
const CMD7:  u16 = 7;   // SELECT_CARD — R1b
const CMD8:  u16 = 0x0408; // SEND_IF_COND — R7 (special encoding)
const CMD12: u16 = 12;  // STOP_TRANSMISSION — R1b
const CMD16: u16 = 16;  // SET_BLOCKLEN — R1
const CMD17: u16 = 17;  // READ_SINGLE_BLOCK — R1 + data
const CMD18: u16 = 18;  // READ_MULTIPLE_BLOCK — R1 + data
const CMD24: u16 = 24;  // WRITE_BLOCK — R1 + data
const CMD25: u16 = 25;  // WRITE_MULTIPLE_BLOCK — R1 + data
const CMD55: u16 = 55;  // APP_CMD (prefix for ACMD) — R1

/// ACMD6 (0x40 | 6): SET_BUS_WIDTH — R1
const ACMD6:  u16 = 0x40 | 6;
/// ACMD41 (0x40 | 41): SD_SEND_OP_COND — R3
const ACMD41: u16 = 0x40 | 41;

// CMD17/18 SDHC variant: upper bits encode high-capacity block addressing.
// Matches the Renesas driver: `CMD18 | 0x7c00u`.
const CMD17_SDHC: u16 = CMD17 | 0x7C00;
const CMD18_SDHC: u16 = CMD18 | 0x7C00;
const CMD24_SDHC: u16 = CMD24 | 0x7C00;
const CMD25_SDHC: u16 = CMD25 | 0x7C00;

// ---------------------------------------------------------------------------
// Voltage / capacity constants
// ---------------------------------------------------------------------------

/// OCR voltage window: 3.2–3.4 V (matches Deluge hardware, SD_VOLT_3_3).
const OCR_VDD_32_33: u32 = 0x0010_0000;
/// OCR host capacity support: HCS bit (high-capacity) for SDHC/SDXC.
const OCR_HCS:       u32 = 0x4000_0000;
/// OCR power-up status bit: set when card is ready.
const OCR_BUSY:      u32 = 0x8000_0000;

/// CMD8 argument: VHS=1 (2.7–3.6 V) | check pattern 0xAA.
const CMD8_ARG: u32 = 0x0000_01AA;

/// ACMD41 argument: HCS + voltage window.
const ACMD41_ARG: u32 = OCR_HCS | OCR_VDD_32_33;

// ---------------------------------------------------------------------------
// Global card state
// ---------------------------------------------------------------------------

/// RCA (Relative Card Address) — set during CMD3, used for CMD7 etc.
static CARD_RCA: AtomicU16 = AtomicU16::new(0);
/// `true` if the card is SDHC/SDXC (uses block addressing).
static CARD_HC:  AtomicBool = AtomicBool::new(false);
/// `true` once `init()` has completed successfully.
static CARD_READY: AtomicBool = AtomicBool::new(false);

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SdError {
    /// Low-level SDHI hardware error.
    Hardware(SdhiError),
    /// Card not present.
    NoCard,
    /// Unsupported card type (e.g. MMC, old SD v1 without sane CMD8 response).
    UnsupportedCard,
    /// Protocol violation — unexpected response.
    Protocol,
    /// Driver not initialized; call [`init`] first.
    NotInitialized,
}

impl From<SdhiError> for SdError {
    fn from(e: SdhiError) -> Self { SdError::Hardware(e) }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Issue CMD55 (APP_CMD prefix) with the current RCA, then issue `acmd`.
async unsafe fn send_acmd(acmd: u16, arg: u32) -> Result<(), SdError> {
    let rca = CARD_RCA.load(Ordering::Relaxed);
    sdhi::set_arg(SD_PORT, (rca as u32) << 16);
    sdhi::send_cmd(SD_PORT, CMD55).await?;
    sdhi::set_arg(SD_PORT, arg);
    sdhi::send_cmd(SD_PORT, acmd).await?;
    Ok(())
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Initialize the SD card (hardware + protocol).
///
/// Must be called once, with the GIC initialized and SDHI IRQs registered.
/// Safe to call again after a card-swap event.
///
/// # Returns
/// `Ok(())` on success; `Err(SdError)` on any failure.
pub async fn init() -> Result<(), SdError> {
    CARD_READY.store(false, Ordering::Release);

    unsafe {
        sdhi::init(SD_PORT);
        sdhi::register_irqs(SD_PORT);
    }

    // Short delay to allow power-up (hardware needs ~1 ms minimum,
    // awaiting a tiny future yields back to the executor which is enough).
    embassy_time::Timer::after_millis(5).await;

    unsafe {
        // ---- CMD0: reset to IDLE ----
        // No response expected; ignore timeout.
        sdhi::set_arg(SD_PORT, 0);
        let _ = sdhi::send_cmd(SD_PORT, CMD0).await;

        embassy_time::Timer::after_millis(1).await;

        // ---- CMD8: check voltage — determines SD v2 / SDHC capability ----
        sdhi::set_arg(SD_PORT, CMD8_ARG);
        let cmd8_ok = sdhi::send_cmd(SD_PORT, CMD8).await.is_ok();

        // ---- ACMD41: initialize card ----
        // Loop until the card clears the busy bit in OCR (card-power-up done).
        let hcs_arg = if cmd8_ok { ACMD41_ARG } else { OCR_VDD_32_33 };
        let mut retries = 0u32;
        let ocr = loop {
            send_acmd(ACMD41, hcs_arg).await?;
            let ocr = sdhi::read_r1(SD_PORT); // R3 OCR comes via same regs

            // Bit 31 set → card no longer busy (initialization complete)
            if ocr & OCR_BUSY != 0 {
                break ocr;
            }
            retries += 1;
            if retries > 1000 {
                return Err(SdError::UnsupportedCard);
            }
            embassy_time::Timer::after_millis(1).await;
        };

        // Determine high-capacity flag
        let hc = cmd8_ok && (ocr & OCR_HCS != 0);
        CARD_HC.store(hc, Ordering::Release);

        // ---- CMD2: get CID (ignore content, just consume response) ----
        sdhi::set_arg(SD_PORT, 0);
        sdhi::send_cmd(SD_PORT, CMD2).await?;
        let _ = sdhi::read_r2(SD_PORT); // CID — discard

        // ---- CMD3: get RCA ----
        sdhi::set_arg(SD_PORT, 0);
        sdhi::send_cmd(SD_PORT, CMD3).await?;
        let r6 = sdhi::read_r1(SD_PORT);
        // R6 = [31:16] new RCA, [15:0] card status
        let rca = (r6 >> 16) as u16;
        CARD_RCA.store(rca, Ordering::Release);

        // ---- CMD7: select card (transition to Transfer state) ----
        sdhi::set_arg(SD_PORT, (rca as u32) << 16);
        sdhi::send_cmd(SD_PORT, CMD7).await?;

        // ---- ACMD6: set 4-bit bus ----
        // Argument 0x2 = 4-bit, 0x0 = 1-bit.
        send_acmd(ACMD6, 0x2).await?;

        // ---- CMD16: set block length to 512 bytes (SDSC cards) ----
        if !hc {
            sdhi::set_arg(SD_PORT, 512);
            sdhi::send_cmd(SD_PORT, CMD16).await?;
        }

        // ---- Switch to high-speed clock ----
        sdhi::set_clock_fast(SD_PORT);
    }

    CARD_READY.store(true, Ordering::Release);
    Ok(())
}

/// Read a single 512-byte sector at logical block address `lba`.
///
/// # Arguments
/// * `lba`  — sector number (0-based).
/// * `buf`  — destination buffer; must be exactly 512 bytes.
pub async fn read_sector(lba: u32, buf: &mut [u8; 512]) -> Result<(), SdError> {
    if !CARD_READY.load(Ordering::Acquire) {
        return Err(SdError::NotInitialized);
    }

    let addr = lba_to_addr(lba);
    let hc = CARD_HC.load(Ordering::Relaxed);
    let cmd = if hc { CMD17_SDHC } else { CMD17 };

    unsafe {
        sdhi::set_block_count(SD_PORT, 1);
        sdhi::set_arg(SD_PORT, addr);
        sdhi::send_cmd(SD_PORT, cmd).await?;
        sdhi::read_blocks_sw(SD_PORT, buf.as_mut_ptr(), 1).await?;
    }
    Ok(())
}

/// Write a single 512-byte sector at logical block address `lba`.
///
/// # Arguments
/// * `lba`  — sector number (0-based).
/// * `buf`  — source data; must be exactly 512 bytes.
pub async fn write_sector(lba: u32, buf: &[u8; 512]) -> Result<(), SdError> {
    if !CARD_READY.load(Ordering::Acquire) {
        return Err(SdError::NotInitialized);
    }

    let addr = lba_to_addr(lba);
    let hc = CARD_HC.load(Ordering::Relaxed);
    let cmd = if hc { CMD24_SDHC } else { CMD24 };

    unsafe {
        sdhi::set_block_count(SD_PORT, 1);
        sdhi::set_arg(SD_PORT, addr);
        sdhi::send_cmd(SD_PORT, cmd).await?;
        sdhi::write_blocks_sw(SD_PORT, buf.as_ptr(), 1).await?;
    }
    Ok(())
}

/// Read `count` consecutive sectors starting at `lba`.
///
/// Uses CMD18 (READ_MULTIPLE_BLOCK) when `count > 1`, CMD17 otherwise.
///
/// # Arguments
/// * `lba`   — first sector (0-based).
/// * `count` — number of sectors.
/// * `buf`   — destination; must hold exactly `count * 512` bytes.
pub async fn read_sectors(lba: u32, count: u32, buf: &mut [u8]) -> Result<(), SdError> {
    if !CARD_READY.load(Ordering::Acquire) {
        return Err(SdError::NotInitialized);
    }
    if buf.len() < (count as usize) * 512 {
        return Err(SdError::Protocol);
    }
    if count == 0 {
        return Ok(());
    }
    if count == 1 {
        let arr = buf[..512].as_mut_ptr() as *mut [u8; 512];
        return read_sector(lba, unsafe { &mut *arr }).await;
    }

    let addr = lba_to_addr(lba);
    let hc = CARD_HC.load(Ordering::Relaxed);
    let cmd = if hc { CMD18_SDHC } else { CMD18 };

    unsafe {
        sdhi::set_block_count(SD_PORT, count);
        sdhi::set_arg(SD_PORT, addr);
        sdhi::send_cmd(SD_PORT, cmd).await?;
        sdhi::read_blocks_sw(SD_PORT, buf.as_mut_ptr(), count).await?;
        // CMD18 auto-stops via SD_STOP SEC bit; belt-and-suspenders here:
        // if auto-stop already issued by HW, the extra CMD12 will be ignored
        // for well-behaved cards.  Omit to avoid the round-trip on every read.
    }
    Ok(())
}

/// Write `count` consecutive sectors starting at `lba`.
///
/// Uses CMD25 (WRITE_MULTIPLE_BLOCK) when `count > 1`, CMD24 otherwise.
pub async fn write_sectors(lba: u32, count: u32, buf: &[u8]) -> Result<(), SdError> {
    if !CARD_READY.load(Ordering::Acquire) {
        return Err(SdError::NotInitialized);
    }
    if buf.len() < (count as usize) * 512 {
        return Err(SdError::Protocol);
    }
    if count == 0 {
        return Ok(());
    }
    if count == 1 {
        let arr = buf[..512].as_ptr() as *const [u8; 512];
        return write_sector(lba, unsafe { &*arr }).await;
    }

    let addr = lba_to_addr(lba);
    let hc = CARD_HC.load(Ordering::Relaxed);
    let cmd = if hc { CMD25_SDHC } else { CMD25 };

    unsafe {
        sdhi::set_block_count(SD_PORT, count);
        sdhi::set_arg(SD_PORT, addr);
        sdhi::send_cmd(SD_PORT, cmd).await?;
        sdhi::write_blocks_sw(SD_PORT, buf.as_ptr(), count).await?;
    }
    Ok(())
}

/// Returns `true` if `init()` completed successfully and the card is ready.
pub fn is_ready() -> bool {
    CARD_READY.load(Ordering::Acquire)
}

/// Returns `true` if a card is physically present (CD pin).
pub fn is_inserted() -> bool {
    unsafe { sdhi::card_inserted(SD_PORT) }
}

// ---------------------------------------------------------------------------
// Internal: convert LBA to card address
// ---------------------------------------------------------------------------

fn lba_to_addr(lba: u32) -> u32 {
    if CARD_HC.load(Ordering::Relaxed) {
        lba           // SDHC/SDXC: block addressing
    } else {
        lba * 512     // SDSC: byte addressing
    }
}
