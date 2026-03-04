# Deluge Host Demo — Python

A Python port of [`examples/host-demo`](../host-demo) that connects to a
Deluge over USB serial and demonstrates hardware control:

- **Pad gradient** — A rainbow HSV gradient that scrolls across the 16×8 pad grid
- **OLED animation** — A bouncing ball, sine wave, and frame counter rendered to the 128×48 OLED
- **Button / pad / encoder readout** — All input events are printed to the console in real time
- **Gold-knob indicator bars** — Driven up/down by turning the two gold encoders
- **Button LED toggle** — Pressing any button toggles its indicator LED

## Requirements

- Python 3.10+
- `pyserial` (`pip install -r requirements.txt`)
- A Deluge running the USB controller firmware with the CDC serial interface

## Install

```sh
cd examples/host-demo-python
pip install -r requirements.txt
```

## Usage

Auto-detect the Deluge (looks for VID:PID `16D0:0EDA`):

```sh
python demo.py
```

Specify a port manually:

```sh
python demo.py --port COM5          # Windows
python demo.py --port /dev/ttyACM0  # Linux
```

Press **Ctrl-C** to exit cleanly (pads and OLED are cleared on exit).

## Library: `deluge.py`

`deluge.py` is a standalone library you can import in your own scripts:

```python
from deluge import Deluge, PadPressed, EncoderRotated

with Deluge.open() as d:
    major, minor, patch = d.get_version()
    print(f"Firmware {major}.{minor}.{patch}")

    # Set a pad
    d.set_pad(col=0, row=0, r=255, g=0, b=0)

    # Poll for events
    while True:
        event = d.try_read_event()
        if isinstance(event, PadPressed):
            print(f"Pad pressed: col={event.col} row={event.row}")
        elif isinstance(event, EncoderRotated):
            print(f"Encoder {event.id} rotated by {event.delta}")
```

### API summary

| Method | Description |
|--------|-------------|
| `Deluge.open()` | Auto-detect Deluge by VID:PID |
| `Deluge.open_port(name)` | Open a specific port |
| `d.ping()` | Send ping, wait for pong |
| `d.get_version()` | Returns `(major, minor, patch)` |
| `d.set_pad(col, row, r, g, b)` | Set a single pad colour |
| `d.set_all_pads(colors)` | Bulk RGB pad update (144 entries, col-major) |
| `d.clear_pads()` | Turn off all pads |
| `d.update_display(fb)` | Push 768-byte OLED framebuffer |
| `d.clear_display()` | Clear the OLED |
| `d.set_led(index, on)` | Set indicator LED 0–35 |
| `d.set_knob_indicator(which, levels)` | Set 4-LED gold-knob bar |
| `d.set_synced_led(on)` | Toggle rear-panel sync LED |
| `d.set_cv(channel, value)` | Set CV output (channel 0–3, value 0–65535) |
| `d.set_gate(channel, on)` | Set gate output (channel 0–3) |
| `d.try_read_event()` | Non-blocking read; returns event or `None` |
| `d.drain_events()` | Read all buffered events as a list |

### OLED framebuffer helpers

```python
from deluge import (
    oled_new_framebuffer,
    oled_set_pixel,
    oled_draw_filled_circle,
    oled_draw_string,
    OLED_WIDTH, OLED_HEIGHT, OLED_TOPMOST_PIXEL, OLED_VISIBLE_HEIGHT,
)

fb = oled_new_framebuffer()      # 768-byte bytearray, all zeros
oled_set_pixel(fb, x, y)
oled_draw_filled_circle(fb, cx, cy, radius)
oled_draw_string(fb, x, y, "HELLO")   # built-in 5×7 font
d.update_display(fb)
```

> **Note:** The top `OLED_TOPMOST_PIXEL` (5) rows of the framebuffer are hidden
> behind the display bezel.  The visible area starts at row 5.

## Protocol

See [`src/controller/usb_serial.h`](../../src/controller/usb_serial.h) for the
full message specification.
