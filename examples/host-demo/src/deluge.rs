// Deluge USB Serial Protocol Implementation
//
// Message format: [length_lo:1][length_hi:1][type:1][data:N]
// Length = 1 (type byte) + N (data bytes), does NOT include the 2 length bytes.

use anyhow::{bail, Context, Result};
use std::io::{Read, Write};
use std::time::{Duration, Instant};

// ── Message types FROM Deluge TO Host ────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MsgFrom {
    PadPressed = 0x01,
    PadReleased = 0x02,
    ButtonPressed = 0x03,
    ButtonReleased = 0x04,
    EncoderRotated = 0x05,
    EncoderPressed = 0x06,
    EncoderReleased = 0x07,
    Version = 0x10,
    Pong = 0x11,
    Ready = 0x12,
    Error = 0x13,
}

impl MsgFrom {
    fn from_byte(b: u8) -> Option<Self> {
        match b {
            0x01 => Some(Self::PadPressed),
            0x02 => Some(Self::PadReleased),
            0x03 => Some(Self::ButtonPressed),
            0x04 => Some(Self::ButtonReleased),
            0x05 => Some(Self::EncoderRotated),
            0x06 => Some(Self::EncoderPressed),
            0x07 => Some(Self::EncoderReleased),
            0x10 => Some(Self::Version),
            0x11 => Some(Self::Pong),
            0x12 => Some(Self::Ready),
            0x13 => Some(Self::Error),
            _ => None,
        }
    }
}

// ── Message types TO Deluge FROM Host ────────────────────────────────

#[repr(u8)]
enum MsgTo {
    UpdateDisplay = 0x20,
    ClearDisplay = 0x21,
    SetPadRgb = 0x22,
    ClearAllPads = 0x23,
    #[allow(dead_code)]
    SetLed = 0x24,
    #[allow(dead_code)]
    SetCv = 0x25,
    #[allow(dead_code)]
    SetGate = 0x26,
    ClearAllLeds = 0x2A,
    SetAllPads = 0x27,
    SetKnobIndicator = 0x28,
    SetSyncedLed = 0x29,
    GetVersion = 0x30,
    Ping = 0x31,
}

// ── Parsed events from the Deluge ────────────────────────────────────

#[derive(Debug, Clone)]
pub enum DelugeEvent {
    PadPressed { col: u8, row: u8 },
    PadReleased { col: u8, row: u8 },
    ButtonPressed { id: u8 },
    ButtonReleased { id: u8 },
    EncoderRotated { id: u8, delta: i8 },
    EncoderPressed { id: u8 },
    EncoderReleased { id: u8 },
    Version { major: u8, minor: u8, patch: u8 },
    Pong,
    Ready,
    Error(String),
}

// ── Deluge connection ────────────────────────────────────────────────

pub struct Deluge {
    port: Box<dyn serialport::SerialPort>,
}

/// OLED display dimensions
pub const OLED_WIDTH: usize = 128;
pub const OLED_HEIGHT: usize = 48;
pub const OLED_PAGES: usize = OLED_HEIGHT / 8; // 6
pub const OLED_FRAMEBUFFER_SIZE: usize = OLED_PAGES * OLED_WIDTH; // 768
/// Top rows of the framebuffer that are hidden behind the display bezel.
/// Matches OLED_MAIN_TOPMOST_PIXEL in cpu_specific.h.
pub const OLED_TOPMOST_PIXEL: usize = 5;
/// Visible pixel rows (framebuffer rows OLED_TOPMOST_PIXEL..OLED_HEIGHT).
pub const OLED_VISIBLE_HEIGHT: usize = OLED_HEIGHT - OLED_TOPMOST_PIXEL; // 43

/// Pad grid dimensions
pub const PAD_COLS: u8 = 18;
pub const PAD_ROWS: u8 = 8;

impl Deluge {
    /// Auto-detect and open the Deluge serial port.
    pub fn open() -> Result<Self> {
        let ports = serialport::available_ports().context("Failed to list serial ports")?;

        // Look for Deluge VID:PID = 16D0:0EDA
        for port_info in &ports {
            if let serialport::SerialPortType::UsbPort(usb) = &port_info.port_type {
                if usb.vid == 0x16D0 && usb.pid == 0x0EDA {
                    println!("Found Deluge on {}", port_info.port_name);
                    let mut port = serialport::new(&port_info.port_name, 115200)
                        .timeout(Duration::from_millis(500))
                        .open()
                        .with_context(|| {
                            format!("Failed to open serial port {}", port_info.port_name)
                        })?;
                    // Assert DTR so TinyUSB's tud_cdc_connected() returns true
                    if let Err(e) = port.write_data_terminal_ready(true) {
                        eprintln!("Warning: could not set DTR: {e}");
                    }
                    // Small delay for the device to process the line state change
                    std::thread::sleep(Duration::from_millis(100));
                    return Ok(Self { port });
                }
            }
        }

        // Fallback: list available ports and give a helpful message
        if ports.is_empty() {
            bail!("No serial ports found. Is the Deluge connected via USB?");
        }
        println!("Available serial ports:");
        for p in &ports {
            println!("  {} ({:?})", p.port_name, p.port_type);
        }
        bail!("Deluge not found (VID:PID 16D0:0EDA). Is the USB controller firmware flashed?");
    }

    /// Open a specific serial port by name.
    pub fn open_port(name: &str) -> Result<Self> {
        let mut port = serialport::new(name, 115200)
            .timeout(Duration::from_millis(500))
            .open()
            .with_context(|| format!("Failed to open serial port {}", name))?;
        // Assert DTR so TinyUSB's tud_cdc_connected() returns true
        if let Err(e) = port.write_data_terminal_ready(true) {
            eprintln!("Warning: could not set DTR: {e}");
        }
        std::thread::sleep(Duration::from_millis(100));
        Ok(Self { port })
    }

    // ── Low-level send/receive ───────────────────────────────────────

    fn send_raw(&mut self, msg_type: u8, data: &[u8]) -> Result<()> {
        let length = 1u16 + data.len() as u16; // type + data
        let mut buf = Vec::with_capacity(3 + data.len());
        buf.push((length & 0xFF) as u8);
        buf.push((length >> 8) as u8);
        buf.push(msg_type);
        buf.extend_from_slice(data);
        // write_all can return ERROR_SEM_TIMEOUT (121) on Windows CDC — retry on timeout
        let mut written = 0usize;
        let deadline = Instant::now() + Duration::from_secs(2);
        while written < buf.len() {
            match self.port.write(&buf[written..]) {
                Ok(n) => written += n,
                Err(e) if Self::is_timeout(&e) => {
                    if Instant::now() >= deadline {
                        return Err(e).context("Write timed out sending to Deluge");
                    }
                    std::thread::sleep(Duration::from_millis(5));
                    continue;
                }
                Err(e) => return Err(e.into()),
            }
        }
        Ok(())
    }

    /// Check if an I/O error is a timeout (handles Windows ERROR_SEM_TIMEOUT = 121).
    fn is_timeout(e: &std::io::Error) -> bool {
        e.kind() == std::io::ErrorKind::TimedOut
            || e.raw_os_error() == Some(121) // ERROR_SEM_TIMEOUT on Windows
    }

    /// Read exactly `n` bytes, returning `None` if no data arrives at all (timeout).
    /// Once the first byte is received, keeps reading until all bytes arrive.
    fn read_exact_or_timeout(&mut self, buf: &mut [u8]) -> Result<Option<()>> {
        let mut filled = 0usize;
        while filled < buf.len() {
            match self.port.read(&mut buf[filled..]) {
                Ok(0) => bail!("Serial port closed unexpectedly"),
                Ok(n) => filled += n,
                Err(e) if Self::is_timeout(&e) => {
                    if filled == 0 {
                        return Ok(None); // No data at all — clean timeout
                    }
                    // Partial read — keep trying (data is in flight)
                    continue;
                }
                Err(e) => return Err(e.into()),
            }
        }
        Ok(Some(()))
    }

    /// Try to read one message from the port. Returns `None` immediately if no
    /// data is waiting (non-blocking fast-return path via `bytes_to_read`).
    pub fn try_read_event(&mut self) -> Result<Option<DelugeEvent>> {
        // Fast non-blocking check — avoid the 500ms port timeout when idle.
        match self.port.bytes_to_read() {
            Ok(n) if n < 3 => return Ok(None), // need at least 2-byte hdr + 1-byte type
            Err(_) => return Ok(None),
            _ => {}
        }

        // Read 2-byte length header
        let mut hdr = [0u8; 2];
        if self.read_exact_or_timeout(&mut hdr)?.is_none() {
            return Ok(None);
        }

        let msg_len = u16::from_le_bytes(hdr) as usize;
        if msg_len == 0 || msg_len > 1024 {
            bail!("Invalid message length: {msg_len}");
        }

        // Read type + data
        let mut payload = vec![0u8; msg_len];
        self.read_exact_or_timeout(&mut payload)?
            .context("Timeout reading message payload after header")?;

        let msg_type = payload[0];
        let data = &payload[1..];

        let Some(mt) = MsgFrom::from_byte(msg_type) else {
            bail!("Unknown message type: 0x{msg_type:02X}");
        };

        let event = match mt {
            MsgFrom::PadPressed if data.len() >= 2 => DelugeEvent::PadPressed {
                col: data[0],
                row: data[1],
            },
            MsgFrom::PadReleased if data.len() >= 2 => DelugeEvent::PadReleased {
                col: data[0],
                row: data[1],
            },
            MsgFrom::ButtonPressed if data.len() >= 1 => {
                DelugeEvent::ButtonPressed { id: data[0] }
            }
            MsgFrom::ButtonReleased if data.len() >= 1 => {
                DelugeEvent::ButtonReleased { id: data[0] }
            }
            MsgFrom::EncoderRotated if data.len() >= 2 => DelugeEvent::EncoderRotated {
                id: data[0],
                delta: data[1] as i8,
            },
            MsgFrom::EncoderPressed if data.len() >= 1 => {
                DelugeEvent::EncoderPressed { id: data[0] }
            }
            MsgFrom::EncoderReleased if data.len() >= 1 => {
                DelugeEvent::EncoderReleased { id: data[0] }
            }
            MsgFrom::Version if data.len() >= 3 => DelugeEvent::Version {
                major: data[0],
                minor: data[1],
                patch: data[2],
            },
            MsgFrom::Pong => DelugeEvent::Pong,
            MsgFrom::Ready => DelugeEvent::Ready,
            MsgFrom::Error => {
                let msg = String::from_utf8_lossy(data).to_string();
                DelugeEvent::Error(msg)
            }
            _ => bail!("Message 0x{msg_type:02X} with unexpected data length {}", data.len()),
        };

        Ok(Some(event))
    }

    // ── High-level commands ──────────────────────────────────────────

    /// Send a ping and wait for pong.
    pub fn ping(&mut self) -> Result<()> {
        self.send_raw(MsgTo::Ping as u8, &[])?;
        // Wait up to 1 second for pong (using multiple reads at the port's base timeout)
        let deadline = Instant::now() + Duration::from_secs(1);
        loop {
            if Instant::now() >= deadline {
                bail!("Ping timed out - no pong received");
            }
            match self.try_read_event()? {
                Some(DelugeEvent::Pong) => return Ok(()),
                Some(_) => continue, // skip other events
                None => continue,    // timeout on this read, retry until deadline
            }
        }
    }

    /// Request and return the firmware version.
    pub fn get_version(&mut self) -> Result<(u8, u8, u8)> {
        self.send_raw(MsgTo::GetVersion as u8, &[])?;
        let deadline = Instant::now() + Duration::from_secs(1);
        loop {
            if Instant::now() >= deadline {
                bail!("Get version timed out");
            }
            match self.try_read_event()? {
                Some(DelugeEvent::Version {
                    major,
                    minor,
                    patch,
                }) => return Ok((major, minor, patch)),
                Some(_) => continue,
                None => continue,
            }
        }
    }

    /// Set a single pad to an RGB color.
    #[allow(dead_code)]
    pub fn set_pad(&mut self, col: u8, row: u8, r: u8, g: u8, b: u8) -> Result<()> {
        self.send_raw(MsgTo::SetPadRgb as u8, &[col, row, r, g, b])
    }

    /// Set an orange indicator LED on or off (index 0–35).
    pub fn set_led(&mut self, index: u8, on: bool) -> Result<()> {
        self.send_raw(MsgTo::SetLed as u8, &[index, on as u8])
    }

    /// Set the 4-LED gold knob indicator bar.
    /// `which`: 0 = bottom knob (MOD_0), 1 = top knob (MOD_1).
    /// `levels`: brightness for each LED from bottom to top (0 = off, 255 = full).
    pub fn set_knob_indicator(&mut self, which: u8, levels: &[u8; 4]) -> Result<()> {
        self.send_raw(
            MsgTo::SetKnobIndicator as u8,
            &[which, levels[0], levels[1], levels[2], levels[3]],
        )
    }

    /// Set the tempo external-synced GPIO indicator LED.
    pub fn set_synced_led(&mut self, on: bool) -> Result<()> {
        self.send_raw(MsgTo::SetSyncedLed as u8, &[on as u8])
    }

    /// Turn off all button indicator LEDs, both gold knob indicator bars, and the synced LED.
    pub fn clear_all_leds(&mut self) -> Result<()> {
        self.send_raw(MsgTo::ClearAllLeds as u8, &[])
    }

    /// Clear all pads.
    pub fn clear_pads(&mut self) -> Result<()> {
        self.send_raw(MsgTo::ClearAllPads as u8, &[])
    }

    /// Send a full OLED framebuffer (768 bytes, 6 pages x 128 columns).
    /// Each byte represents 8 vertical pixels (LSB = top of page).
    pub fn update_display(&mut self, framebuffer: &[u8; OLED_FRAMEBUFFER_SIZE]) -> Result<()> {
        self.send_raw(MsgTo::UpdateDisplay as u8, framebuffer)
    }

    /// Clear the OLED display.
    pub fn clear_display(&mut self) -> Result<()> {
        self.send_raw(MsgTo::ClearDisplay as u8, &[])
    }

    /// Send all pad colours in one bulk message.
    /// `colors` must be col-major: for col in 0..PAD_COLS, row in 0..PAD_ROWS → [r, g, b].
    pub fn set_all_pads(&mut self, colors: &[[u8; 3]; PAD_COLS as usize * PAD_ROWS as usize]) -> Result<()> {
        let mut buf = [0u8; PAD_COLS as usize * PAD_ROWS as usize * 3];
        for (i, rgb) in colors.iter().enumerate() {
            buf[i * 3] = rgb[0];
            buf[i * 3 + 1] = rgb[1];
            buf[i * 3 + 2] = rgb[2];
        }
        self.send_raw(MsgTo::SetAllPads as u8, &buf)
    }
}
