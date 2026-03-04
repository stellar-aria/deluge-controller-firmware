"""
Deluge Host Demo (Python port of examples/host-demo)

Demonstrates:
  • Rainbow HSV gradient that scrolls across the 16×8 pad grid over time
  • OLED animation — bouncing ball, sine wave, and frame counter
  • Button / pad / encoder event readout printed to the console
  • Gold-knob indicator bars driven by encoder rotation
  • Button LED toggle on button press
  • Tempo encoder button toggles the external-sync LED

Usage
-----
Auto-detect the Deluge (VID:PID 16D0:0EDA):

    python demo.py

Or specify a port manually:

    python demo.py --port COM5          # Windows
    python demo.py --port /dev/ttyACM0  # Linux
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time

from deluge import (
    Deluge,
    DelugeEvent,
    ButtonPressed,
    ButtonReleased,
    EncoderPressed,
    EncoderReleased,
    EncoderRotated,
    ErrorEvent,
    PadPressed,
    PadReleased,
    Pong,
    Ready,
    VersionEvent,
    OLED_FRAMEBUFFER_SIZE,
    OLED_HEIGHT,
    OLED_TOPMOST_PIXEL,
    OLED_VISIBLE_HEIGHT,
    OLED_WIDTH,
    PAD_COLS,
    PAD_ROWS,
    oled_draw_filled_circle,
    oled_draw_string,
    oled_new_framebuffer,
    oled_set_pixel,
)

# ---------------------------------------------------------------------------
# Constants — mirror main.rs
# ---------------------------------------------------------------------------

# Encoder IDs for the gold knobs (MOD_0 / MOD_1)
GOLD_KNOB_0_ENC_ID = 2
GOLD_KNOB_1_ENC_ID = 3

# 4-LED indicator bar range
KNOB_MAX = 4

# Tempo encoder button: tempoEncButtonCoord = {4,1} → LED index 13 → PIC id 157
TEMPO_ENC_BUTTON_ID = 144 + 4 + 1 * 9  # 157

# ---------------------------------------------------------------------------
# Demo state
# ---------------------------------------------------------------------------

class DemoState:
    def __init__(self) -> None:
        self.knob_levels: list[int] = [KNOB_MAX // 2, KNOB_MAX // 2]
        self.button_leds: list[bool] = [False] * 36
        self.synced_led: bool = False


# ---------------------------------------------------------------------------
# HSV → RGB helper
# ---------------------------------------------------------------------------

def hsv_to_rgb(h: float, s: float, v: float) -> tuple[int, int, int]:
    """Convert HSV (0-1 each) to (r, g, b) 0-255."""
    h = (h % 1.0) * 6.0
    i = int(h)
    f = h - i
    p = v * (1.0 - s)
    q = v * (1.0 - s * f)
    t = v * (1.0 - s * (1.0 - f))
    sector = i % 6
    if sector == 0:
        r, g, b = v, t, p
    elif sector == 1:
        r, g, b = q, v, p
    elif sector == 2:
        r, g, b = p, v, t
    elif sector == 3:
        r, g, b = p, q, v
    elif sector == 4:
        r, g, b = t, p, v
    else:
        r, g, b = v, p, q
    return (int(r * 255), int(g * 255), int(b * 255))


# ---------------------------------------------------------------------------
# Pad gradient
# ---------------------------------------------------------------------------

def update_pad_gradient(deluge: Deluge, t: float) -> None:
    """
    Send a rainbow HSV gradient to all pads.
    Hue shifts by column and scrolls over time.
    """
    colors: list[tuple[int, int, int]] = []
    for col in range(PAD_COLS):
        for row in range(PAD_ROWS):
            hue   = ((col / PAD_COLS) + t * 0.15) % 1.0
            value = 0.3 + 0.7 * (row / (PAD_ROWS - 1))
            colors.append(hsv_to_rgb(hue, 1.0, value))
    deluge.set_all_pads(colors)


# ---------------------------------------------------------------------------
# OLED animation
# ---------------------------------------------------------------------------

def render_oled_frame(fb: bytearray, t: float, frame: int) -> None:
    """
    Render a frame into *fb* (zeroed first):
      • Bouncing ball
      • Sine wave across the middle
      • Frame counter in the top-left of the visible area
    """
    # Clear
    for i in range(len(fb)):
        fb[i] = 0

    top   = OLED_TOPMOST_PIXEL
    vis_h = OLED_VISIBLE_HEIGHT

    # ── Bouncing ball ────────────────────────────────────────────────
    ball_r = 6
    bx = int((math.sin(t * 45.0) * 0.5 + 0.5) * (OLED_WIDTH - 2 * ball_r) + ball_r)
    by = int(
        top
        + (math.sin(t * 60.0) * 0.5 + 0.5) * (vis_h - 2 * ball_r)
        + ball_r
    )
    oled_draw_filled_circle(fb, bx, by, ball_r)

    # ── Sine wave across the middle ──────────────────────────────────
    for x in range(OLED_WIDTH):
        y = top + vis_h // 2 + int(math.sin(x * 0.08 + t * 3.0) * 10.0)
        if top <= y < OLED_HEIGHT:
            oled_set_pixel(fb, x, y)

    # ── Frame counter ────────────────────────────────────────────────
    oled_draw_string(fb, 1, top + 1, f"F:{frame}")


# ---------------------------------------------------------------------------
# Event handling
# ---------------------------------------------------------------------------

def print_event(event: DelugeEvent) -> None:
    if isinstance(event, PadPressed):
        print(f"[PAD]     pressed  col={event.col} row={event.row}")
    elif isinstance(event, PadReleased):
        print(f"[PAD]     released col={event.col} row={event.row}")
    elif isinstance(event, ButtonPressed):
        print(f"[BUTTON]  pressed  id={event.id}")
    elif isinstance(event, ButtonReleased):
        print(f"[BUTTON]  released id={event.id}")
    elif isinstance(event, EncoderRotated):
        print(f"[ENCODER] rotated  id={event.id} delta={event.delta}")
    elif isinstance(event, EncoderPressed):
        print(f"[ENCODER] pressed  id={event.id}")
    elif isinstance(event, EncoderReleased):
        print(f"[ENCODER] released id={event.id}")
    elif isinstance(event, VersionEvent):
        print(f"[VERSION] {event.major}.{event.minor}.{event.patch}")
    elif isinstance(event, Pong):
        print("[PONG]")
    elif isinstance(event, Ready):
        print("[READY]")
    elif isinstance(event, ErrorEvent):
        print(f"[ERROR]   {event.message}")


def _apply_knob_indicator(deluge: Deluge, state: DemoState, knob: int) -> None:
    level = state.knob_levels[knob]
    levels = (
        255 if level > 0 else 0,
        255 if level > 1 else 0,
        255 if level > 2 else 0,
        255 if level > 3 else 0,
    )
    deluge.set_knob_indicator(knob, levels)


def handle_event(deluge: Deluge, state: DemoState, event: DelugeEvent) -> None:
    """Apply interactive LED changes triggered by an event (mirrors main.rs)."""
    if isinstance(event, EncoderRotated):
        if event.id == GOLD_KNOB_0_ENC_ID:
            knob = 0
        elif event.id == GOLD_KNOB_1_ENC_ID:
            knob = 1
        else:
            return
        state.knob_levels[knob] = max(0, min(KNOB_MAX, state.knob_levels[knob] + event.delta))
        _apply_knob_indicator(deluge, state, knob)

    elif isinstance(event, ButtonPressed):
        bid = event.id
        # 144–179: PIC button range; skip gold-knob push buttons (162, 171)
        if 144 <= bid < 180 and bid not in (162, 171):
            led = bid - 144
            state.button_leds[led] = not state.button_leds[led]
            deluge.set_led(led, state.button_leds[led])
            if bid == TEMPO_ENC_BUTTON_ID:
                state.synced_led = not state.synced_led
                deluge.set_synced_led(state.synced_led)


def drain_events(deluge: Deluge, state: DemoState) -> None:
    while True:
        event = deluge.try_read_event()
        if event is None:
            break
        print_event(event)
        handle_event(deluge, state, event)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        prog="demo.py",
        description="Demo: gradient pads, OLED animation, button readout",
    )
    parser.add_argument(
        "--port", "-p",
        metavar="PORT",
        help="Serial port name (auto-detects if omitted)",
    )
    args = parser.parse_args()

    # ── Connect ──────────────────────────────────────────────────────
    if args.port:
        deluge = Deluge.open_port(args.port)
    else:
        deluge = Deluge.open()

    # Ctrl-C graceful shutdown
    running = True
    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, _stop)

    try:
        # Drain any pending data from a previous session
        print("Draining any pending data...")
        for event in deluge.drain_events():
            print(f"  (draining: {event})")

        print("Pinging Deluge...")
        try:
            deluge.ping()
            print("Pong received!")
        except Exception as exc:
            print(f"Ping failed: {exc}")
            print("Continuing anyway to test event reading...")

        try:
            major, minor, patch = deluge.get_version()
            print(f"Firmware version: {major}.{minor}.{patch}")
        except Exception as exc:
            print(f"Could not get version: {exc}")

        start = time.monotonic()
        frame = 0
        oled_fb = oled_new_framebuffer()

        state = DemoState()
        _apply_knob_indicator(deluge, state, 0)
        _apply_knob_indicator(deluge, state, 1)

        print("\nRunning demo (Ctrl-C to exit)...")
        print("  - Pads: rainbow gradient that scrolls over time")
        print("  - OLED: bouncing ball + frame counter")
        print("  - Input events printed to console\n")

        while running:
            t = time.monotonic() - start

            # 1. Animated pad gradient
            update_pad_gradient(deluge, t)

            # 2. OLED animation
            render_oled_frame(oled_fb, t, frame)
            deluge.update_display(oled_fb)

            # 3. Read and print input events
            drain_events(deluge, state)

            frame += 1
            time.sleep(0.016)  # ~60 fps

    finally:
        print("\nCleaning up...")
        try:
            deluge.clear_pads()
            deluge.clear_display()
            for i in range(36):
                deluge.set_led(i, False)
            deluge.set_knob_indicator(0, (0, 0, 0, 0))
            deluge.set_knob_indicator(1, (0, 0, 0, 0))
            deluge.set_synced_led(False)
        except Exception as exc:
            print(f"Warning during cleanup: {exc}")
        deluge.close()
        print("Done.")


if __name__ == "__main__":
    main()
