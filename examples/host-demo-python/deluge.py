"""
Deluge USB Serial Protocol — Python library

Message framing:
    [length_lo : 1][length_hi : 1][type : 1][data : N]
    length = 1 (type) + N (data), NOT including the 2 length bytes.

See src/controller/usb_serial.h for the canonical message type definitions.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DELUGE_VID = 0x16D0
DELUGE_PID = 0x0EDA

# OLED display dimensions
OLED_WIDTH: int = 128
OLED_HEIGHT: int = 48
OLED_PAGES: int = OLED_HEIGHT // 8              # 6
OLED_FRAMEBUFFER_SIZE: int = OLED_PAGES * OLED_WIDTH  # 768
# Top rows hidden behind the display bezel (matches OLED_MAIN_TOPMOST_PIXEL)
OLED_TOPMOST_PIXEL: int = 5
OLED_VISIBLE_HEIGHT: int = OLED_HEIGHT - OLED_TOPMOST_PIXEL  # 43

# Pad grid dimensions
PAD_COLS: int = 18
PAD_ROWS: int = 8

# ---------------------------------------------------------------------------
# Message types
# ---------------------------------------------------------------------------

class MsgFrom(IntEnum):
    """Messages FROM Deluge TO Host."""
    PAD_PRESSED      = 0x01
    PAD_RELEASED     = 0x02
    BUTTON_PRESSED   = 0x03
    BUTTON_RELEASED  = 0x04
    ENCODER_ROTATED  = 0x05
    ENCODER_PRESSED  = 0x06
    ENCODER_RELEASED = 0x07
    VERSION          = 0x10
    PONG             = 0x11
    READY            = 0x12
    ERROR            = 0x13


class MsgTo(IntEnum):
    """Messages TO Deluge FROM Host."""
    UPDATE_DISPLAY    = 0x20
    CLEAR_DISPLAY     = 0x21
    SET_PAD_RGB       = 0x22
    CLEAR_ALL_PADS    = 0x23
    SET_LED           = 0x24
    SET_CV            = 0x25
    SET_GATE          = 0x26
    SET_ALL_PADS      = 0x27
    SET_KNOB_INDICATOR = 0x28
    SET_SYNCED_LED    = 0x29
    GET_VERSION       = 0x30
    PING              = 0x31

# ---------------------------------------------------------------------------
# Event types
# ---------------------------------------------------------------------------

@dataclass
class PadPressed:
    col: int
    row: int

@dataclass
class PadReleased:
    col: int
    row: int

@dataclass
class ButtonPressed:
    id: int

@dataclass
class ButtonReleased:
    id: int

@dataclass
class EncoderRotated:
    id: int
    delta: int  # signed; positive = clockwise

@dataclass
class EncoderPressed:
    id: int

@dataclass
class EncoderReleased:
    id: int

@dataclass
class VersionEvent:
    major: int
    minor: int
    patch: int

@dataclass
class Pong:
    pass

@dataclass
class Ready:
    pass

@dataclass
class ErrorEvent:
    message: str

# Union type alias for type hints
DelugeEvent = (
    PadPressed | PadReleased |
    ButtonPressed | ButtonReleased |
    EncoderRotated | EncoderPressed | EncoderReleased |
    VersionEvent | Pong | Ready | ErrorEvent
)

# ---------------------------------------------------------------------------
# Deluge connection
# ---------------------------------------------------------------------------

class DelugeError(Exception):
    """Raised for protocol-level errors."""


class Deluge:
    """
    Connection to a Deluge over USB CDC serial.

    Usage::

        with Deluge.open() as d:
            d.ping()
            d.set_pad(0, 0, 255, 0, 0)
            event = d.try_read_event()
    """

    def __init__(self, port: serial.Serial) -> None:
        self._port = port

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    @classmethod
    def open(cls) -> "Deluge":
        """
        Auto-detect and connect to the Deluge (VID:PID 16D0:0EDA).
        Raises RuntimeError if not found.
        """
        ports = list(serial.tools.list_ports.comports())

        for info in ports:
            if info.vid == DELUGE_VID and info.pid == DELUGE_PID:
                print(f"Found Deluge on {info.device}")
                return cls._open_device(info.device)

        if not ports:
            raise RuntimeError(
                "No serial ports found. Is the Deluge connected via USB?"
            )

        print("Available serial ports:")
        for p in ports:
            print(f"  {p.device}  vid={p.vid:#06x} pid={p.pid:#06x}  desc={p.description}")
        raise RuntimeError(
            "Deluge not found (VID:PID 16D0:0EDA). "
            "Is the USB controller firmware flashed?"
        )

    @classmethod
    def open_port(cls, name: str) -> "Deluge":
        """Connect to a specific serial port by name."""
        return cls._open_device(name)

    @classmethod
    def _open_device(cls, name: str) -> "Deluge":
        port = serial.Serial(
            name,
            baudrate=115_200,
            timeout=0.5,       # 500 ms read timeout (matches Rust)
        )
        # Assert DTR so TinyUSB's tud_cdc_connected() returns True.
        try:
            port.dtr = True
        except Exception as exc:
            print(f"Warning: could not set DTR: {exc}")
        # Small delay for the device to process the line state change.
        time.sleep(0.1)
        return cls(port)

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def __enter__(self) -> "Deluge":
        return self

    def __exit__(self, *_) -> None:
        self.close()

    def close(self) -> None:
        """Close the serial port."""
        if self._port and self._port.is_open:
            self._port.close()

    # ------------------------------------------------------------------
    # Low-level send / receive
    # ------------------------------------------------------------------

    def _send_raw(self, msg_type: int, data: bytes | bytearray = b"") -> None:
        """
        Frame and transmit one message.

        Wire layout: [length_lo][length_hi][type][data...]
        where length = 1 + len(data).
        """
        length = 1 + len(data)
        frame = struct.pack("<H", length) + bytes([msg_type]) + bytes(data)

        written = 0
        deadline = time.monotonic() + 2.0
        while written < len(frame):
            try:
                n = self._port.write(frame[written:])
                if n:
                    written += n
            except serial.SerialTimeoutException:
                if time.monotonic() >= deadline:
                    raise DelugeError("Write timed out sending to Deluge")
                time.sleep(0.005)

    def _read_exact(self, n: int, allow_timeout_on_first: bool = False) -> Optional[bytes]:
        """
        Read exactly *n* bytes.
        Returns None if no data arrives at all (clean timeout) and
        allow_timeout_on_first is True; otherwise raises on timeout.
        """
        buf = bytearray()
        first = True
        while len(buf) < n:
            chunk = self._port.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
                first = False
            else:
                # No data returned within the port timeout
                if first and allow_timeout_on_first:
                    return None
                if not first:
                    # Partial read — keep retrying
                    continue
                return None
        return bytes(buf)

    def try_read_event(self) -> Optional[DelugeEvent]:
        """
        Non-blocking read of one event.  Returns None if no data is available.
        """
        # Fast check — avoid waiting for the full port timeout when idle.
        if self._port.in_waiting < 3:
            return None

        # Read 2-byte length header
        hdr = self._read_exact(2, allow_timeout_on_first=True)
        if hdr is None:
            return None
        (msg_len,) = struct.unpack("<H", hdr)

        if msg_len == 0 or msg_len > 1024:
            raise DelugeError(f"Invalid message length: {msg_len}")

        payload = self._read_exact(msg_len)
        if payload is None:
            raise DelugeError("Timeout reading message payload after header")

        msg_type = payload[0]
        data = payload[1:]

        try:
            mt = MsgFrom(msg_type)
        except ValueError:
            raise DelugeError(f"Unknown message type: {msg_type:#04x}")

        if mt == MsgFrom.PAD_PRESSED and len(data) >= 2:
            return PadPressed(col=data[0], row=data[1])
        elif mt == MsgFrom.PAD_RELEASED and len(data) >= 2:
            return PadReleased(col=data[0], row=data[1])
        elif mt == MsgFrom.BUTTON_PRESSED and len(data) >= 1:
            return ButtonPressed(id=data[0])
        elif mt == MsgFrom.BUTTON_RELEASED and len(data) >= 1:
            return ButtonReleased(id=data[0])
        elif mt == MsgFrom.ENCODER_ROTATED and len(data) >= 2:
            delta = struct.unpack("b", bytes([data[1]]))[0]  # signed byte
            return EncoderRotated(id=data[0], delta=delta)
        elif mt == MsgFrom.ENCODER_PRESSED and len(data) >= 1:
            return EncoderPressed(id=data[0])
        elif mt == MsgFrom.ENCODER_RELEASED and len(data) >= 1:
            return EncoderReleased(id=data[0])
        elif mt == MsgFrom.VERSION and len(data) >= 3:
            return VersionEvent(major=data[0], minor=data[1], patch=data[2])
        elif mt == MsgFrom.PONG:
            return Pong()
        elif mt == MsgFrom.READY:
            return Ready()
        elif mt == MsgFrom.ERROR:
            return ErrorEvent(message=data.decode("utf-8", errors="replace"))
        else:
            raise DelugeError(
                f"Message {msg_type:#04x} with unexpected data length {len(data)}"
            )

    # ------------------------------------------------------------------
    # High-level commands ← matches Rust Deluge impl methods
    # ------------------------------------------------------------------

    def ping(self, timeout: float = 1.0) -> None:
        """Send a ping and wait for a pong response."""
        self._send_raw(MsgTo.PING)
        deadline = time.monotonic() + timeout
        while True:
            if time.monotonic() >= deadline:
                raise DelugeError("Ping timed out — no pong received")
            event = self.try_read_event()
            if isinstance(event, Pong):
                return

    def get_version(self, timeout: float = 1.0) -> tuple[int, int, int]:
        """Request firmware version; returns (major, minor, patch)."""
        self._send_raw(MsgTo.GET_VERSION)
        deadline = time.monotonic() + timeout
        while True:
            if time.monotonic() >= deadline:
                raise DelugeError("get_version timed out")
            event = self.try_read_event()
            if isinstance(event, VersionEvent):
                return (event.major, event.minor, event.patch)

    def set_pad(self, col: int, row: int, r: int, g: int, b: int) -> None:
        """Set a single pad LED colour (col 0-17, row 0-7)."""
        self._send_raw(MsgTo.SET_PAD_RGB, bytes([col, row, r, g, b]))

    def set_all_pads(self, colors: list[tuple[int, int, int]]) -> None:
        """
        Bulk RGB pad update.

        *colors* must be a flat col-major list of (r, g, b) tuples with
        PAD_COLS * PAD_ROWS = 144 entries:
            colors[col * PAD_ROWS + row] = (r, g, b)
        """
        if len(colors) != PAD_COLS * PAD_ROWS:
            raise ValueError(
                f"set_all_pads expects {PAD_COLS * PAD_ROWS} entries, got {len(colors)}"
            )
        buf = bytearray(PAD_COLS * PAD_ROWS * 3)
        for i, (r, g, b) in enumerate(colors):
            buf[i * 3]     = r
            buf[i * 3 + 1] = g
            buf[i * 3 + 2] = b
        self._send_raw(MsgTo.SET_ALL_PADS, buf)

    def clear_pads(self) -> None:
        """Turn off all pad LEDs."""
        self._send_raw(MsgTo.CLEAR_ALL_PADS)

    def update_display(self, framebuffer: bytes | bytearray) -> None:
        """
        Push a full OLED framebuffer.

        *framebuffer* must be exactly OLED_FRAMEBUFFER_SIZE (768) bytes.
        Encoding: page-major (6 pages × 128 columns), LSB of each byte = top
        pixel of that page column.
        """
        if len(framebuffer) != OLED_FRAMEBUFFER_SIZE:
            raise ValueError(
                f"framebuffer must be {OLED_FRAMEBUFFER_SIZE} bytes, "
                f"got {len(framebuffer)}"
            )
        self._send_raw(MsgTo.UPDATE_DISPLAY, framebuffer)

    def clear_display(self) -> None:
        """Clear the OLED display."""
        self._send_raw(MsgTo.CLEAR_DISPLAY)

    def set_led(self, index: int, on: bool) -> None:
        """Set an indicator LED on or off (index 0–35)."""
        self._send_raw(MsgTo.SET_LED, bytes([index, int(on)]))

    def set_knob_indicator(self, which: int, levels: tuple[int, int, int, int]) -> None:
        """
        Set the 4-LED gold-knob indicator bar.

        *which*: 0 = MOD_0 (bottom knob), 1 = MOD_1 (top knob).
        *levels*: brightness for each LED segment (0 = off, 255 = full).
        """
        self._send_raw(MsgTo.SET_KNOB_INDICATOR, bytes([which, *levels]))

    def set_synced_led(self, on: bool) -> None:
        """Control the external-sync LED on the rear panel."""
        self._send_raw(MsgTo.SET_SYNCED_LED, bytes([int(on)]))

    def set_cv(self, channel: int, value: int) -> None:
        """
        Set a CV output voltage.

        *channel*: 0–3.
        *value*: 0–65535, linear across 0–10 V.
        """
        self._send_raw(MsgTo.SET_CV, bytes([channel, (value >> 8) & 0xFF, value & 0xFF]))

    def set_gate(self, channel: int, on: bool) -> None:
        """Set a gate output (+5 V when on). channel: 0–3."""
        self._send_raw(MsgTo.SET_GATE, bytes([channel, int(on)]))

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    def drain_events(self) -> list[DelugeEvent]:
        """Read and return all currently buffered events."""
        events: list[DelugeEvent] = []
        while True:
            event = self.try_read_event()
            if event is None:
                break
            events.append(event)
        return events


# ---------------------------------------------------------------------------
# OLED framebuffer helpers
# ---------------------------------------------------------------------------

def oled_set_pixel(fb: bytearray, x: int, y: int) -> None:
    """Set pixel (x, y) in a page-encoded OLED framebuffer."""
    if x < 0 or x >= OLED_WIDTH or y < 0 or y >= OLED_HEIGHT:
        return
    page = y // 8
    bit  = y % 8
    fb[page * OLED_WIDTH + x] |= 1 << bit


def oled_clear_pixel(fb: bytearray, x: int, y: int) -> None:
    """Clear pixel (x, y) in a page-encoded OLED framebuffer."""
    if x < 0 or x >= OLED_WIDTH or y < 0 or y >= OLED_HEIGHT:
        return
    page = y // 8
    bit  = y % 8
    fb[page * OLED_WIDTH + x] &= ~(1 << bit)


def oled_new_framebuffer() -> bytearray:
    """Return a zeroed OLED framebuffer."""
    return bytearray(OLED_FRAMEBUFFER_SIZE)


def oled_draw_filled_circle(fb: bytearray, cx: int, cy: int, r: int) -> None:
    """Draw a filled circle at (cx, cy) with radius r."""
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= r * r:
                oled_set_pixel(fb, cx + dx, cy + dy)


def oled_draw_string(fb: bytearray, x0: int, y0: int, text: str) -> None:
    """Draw a string using the built-in 5×7 bitmap font."""
    x = x0
    for ch in text:
        _oled_draw_char(fb, x, y0, ch)
        x += 6  # 5 pixels + 1 gap


def _oled_draw_char(fb: bytearray, x0: int, y0: int, ch: str) -> None:
    glyph = _FONT_5X7.get(ch.upper())
    if glyph is None:
        return
    for dx, col_bits in enumerate(glyph):
        for bit in range(7):
            if col_bits & (1 << bit):
                oled_set_pixel(fb, x0 + dx, y0 + bit)


# 5×7 bitmap font — each char is 5 column bytes (bit 0 = top row)
_FONT_5X7: dict[str, tuple[int, int, int, int, int]] = {
    '0': (0x3E, 0x51, 0x49, 0x45, 0x3E),
    '1': (0x00, 0x42, 0x7F, 0x40, 0x00),
    '2': (0x42, 0x61, 0x51, 0x49, 0x46),
    '3': (0x21, 0x41, 0x45, 0x4B, 0x31),
    '4': (0x18, 0x14, 0x12, 0x7F, 0x10),
    '5': (0x27, 0x45, 0x45, 0x45, 0x39),
    '6': (0x3C, 0x4A, 0x49, 0x49, 0x30),
    '7': (0x01, 0x71, 0x09, 0x05, 0x03),
    '8': (0x36, 0x49, 0x49, 0x49, 0x36),
    '9': (0x06, 0x49, 0x49, 0x29, 0x1E),
    ':': (0x00, 0x36, 0x36, 0x00, 0x00),
    'F': (0x7F, 0x09, 0x09, 0x09, 0x01),
    'P': (0x7F, 0x09, 0x09, 0x09, 0x06),
    'R': (0x7F, 0x09, 0x19, 0x29, 0x46),
    'E': (0x7F, 0x49, 0x49, 0x49, 0x41),
    'S': (0x46, 0x49, 0x49, 0x49, 0x31),
    'D': (0x7F, 0x41, 0x41, 0x22, 0x1C),
    'B': (0x7F, 0x49, 0x49, 0x49, 0x36),
    'A': (0x7E, 0x11, 0x11, 0x11, 0x7E),
    'L': (0x7F, 0x40, 0x40, 0x40, 0x40),
    'O': (0x3E, 0x41, 0x41, 0x41, 0x3E),
    'N': (0x7F, 0x04, 0x08, 0x10, 0x7F),
    'T': (0x01, 0x01, 0x7F, 0x01, 0x01),
    ' ': (0x00, 0x00, 0x00, 0x00, 0x00),
    'I': (0x00, 0x41, 0x7F, 0x41, 0x00),
    'C': (0x3E, 0x41, 0x41, 0x41, 0x22),
    'G': (0x3E, 0x41, 0x49, 0x49, 0x3A),
    'H': (0x7F, 0x08, 0x08, 0x08, 0x7F),
    'K': (0x7F, 0x08, 0x14, 0x22, 0x41),
    'M': (0x7F, 0x02, 0x0C, 0x02, 0x7F),
    'U': (0x3F, 0x40, 0x40, 0x40, 0x3F),
    'V': (0x1F, 0x20, 0x40, 0x20, 0x1F),
    'W': (0x3F, 0x40, 0x38, 0x40, 0x3F),
    'X': (0x63, 0x14, 0x08, 0x14, 0x63),
    'Y': (0x07, 0x08, 0x70, 0x08, 0x07),
    'Z': (0x61, 0x51, 0x49, 0x45, 0x43),
}
