# Deluge Host Demo

A Rust application that connects to a Deluge over USB serial and demonstrates control of the hardware:

- **Pad gradient** — A rainbow HSV gradient that scrolls across the 16×8 pad grid over time
- **OLED animation** — A bouncing ball, sine wave, and frame counter rendered to the 128×48 OLED
- **Button/pad/encoder readout** — All input events are printed to the console in real time

## Requirements

- Rust toolchain (1.70+)
- A Deluge running the USB controller firmware with the CDC serial interface

## Building

```sh
cd contrib/host-demo
cargo build --release
```

## Usage

Auto-detect the Deluge (looks for VID:PID `16D0:0EDA`):

```sh
cargo run --release
```

Or specify a port manually:

```sh
cargo run --release -- --port COM5       # Windows
cargo run --release -- --port /dev/ttyACM0  # Linux
```

Press **Ctrl-C** to exit cleanly (pads and OLED are cleared on exit).

## Protocol

The demo communicates using the Deluge USB serial protocol:

| Direction | Format |
|-----------|--------|
| Wire frame | `[length_lo][length_hi][type][data...]` |
| Length | Covers type + data (not the 2 length bytes) |

See `src/controller/usb_serial.h` in the firmware for the full message type definitions.
