#! /usr/bin/env python3
"""
MIDI loopback test.

Sends MIDI messages to the Deluge via USB and verifies the same bytes are
echoed back. The firmware bridges USB→DIN and DIN→USB, so a physical wire
from the DIN TX pin to the DIN RX pin completes the loop.

Hardware setup: solder/jumper SCIF0 TX (port 6 pin 15) to SCIF0 RX (port 6
pin 14), or use the Deluge's 5-pin DIN connectors with a standard MIDI
loopback cable.
"""

import sys
import time
import argparse
import util

# Time (seconds) to wait for each echoed message before declaring failure.
DEFAULT_TIMEOUT_S = 0.5


# (raw_midi_bytes, human_label)
CHANNEL_MESSAGES = [
    ([0x90, 0x3C, 0x64], "Note On   ch1  C4  vel=100"),
    ([0x80, 0x3C, 0x00], "Note Off  ch1  C4"),
    ([0xB0, 0x07, 0x7F], "CC        ch1  #7  val=127"),
    ([0xC0, 0x05],       "Prog Chg  ch1  #5"),
    ([0xD0, 0x40],       "Chan AT   ch1  val=64"),
    ([0xE0, 0x00, 0x40], "Pitch Bnd ch1  center"),
    ([0xA0, 0x3C, 0x30], "Poly AT   ch1  C4  val=48"),
    ([0x91, 0x48, 0x50], "Note On   ch2  C5  vel=80"),
    ([0x81, 0x48, 0x00], "Note Off  ch2  C5"),
    ([0xBF, 0x7B, 0x00], "CC        ch16 #123 all-notes-off"),
]

REALTIME_MESSAGES = [
    ([0xF8], "Clock"),
    ([0xFA], "Start"),
    ([0xFB], "Continue"),
    ([0xFC], "Stop"),
]


def drain(midiin, duration_s=0.15):
    """Discard stale messages that arrived before the test started."""
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        midiin.get_message()
        time.sleep(0.005)


def recv_message(midiin, timeout_s):
    """Poll until one MIDI message arrives or the timeout expires."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        result = midiin.get_message()
        if result:
            msg, _ = result
            return list(msg)
        time.sleep(0.005)
    return None


def run_tests(midiout, midiin, include_realtime, timeout_s):
    if include_realtime:
        # un-suppress timing messages so we can receive 0xF8/0xFA/0xFB/0xFC
        midiin.ignore_types(sysex=True, timing=False, active_sense=True)
        messages = CHANNEL_MESSAGES + REALTIME_MESSAGES
    else:
        midiin.ignore_types(sysex=True, timing=True, active_sense=True)
        messages = CHANNEL_MESSAGES

    drain(midiin)

    passed = 0
    failed = 0

    for sent, label in messages:
        midiout.send_message(sent)
        received = recv_message(midiin, timeout_s)

        if received is None:
            print(f"  FAIL  {label}  —  no response (timeout {timeout_s:.2f}s)")
            failed += 1
        elif received == sent:
            print(f"  PASS  {label}  —  {[hex(b) for b in sent]}")
            passed += 1
        else:
            sent_hex = [hex(b) for b in sent]
            recv_hex = [hex(b) for b in received]
            print(f"  FAIL  {label}  —  sent {sent_hex}, got {recv_hex}")
            failed += 1

        # Brief gap so the firmware task loop can flush DIN TX before next message.
        time.sleep(0.02)

    total = passed + failed
    print(f"\n{passed}/{total} passed", end="")
    if failed:
        print(f", {failed} FAILED")
    else:
        print()

    return failed == 0


def argparser():
    parser = argparse.ArgumentParser(
        prog="midi-loopback",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="MIDI loopback test: sends messages to the Deluge and checks they come back intact.",
        epilog="""\nusage examples:
  dbt midi-loopback
  dbt midi-loopback --realtime
  dbt midi-loopback --timeout 1.0 3 3""",
        exit_on_error=False,
    )
    parser.group = "Development"
    parser.add_argument(
        "ports",
        nargs="*",
        type=int,
        help=(
            "MIDI output and input port numbers (e.g. '3 3'). "
            "If omitted, auto-detects Deluge ports. "
            "If one number is given, it is used for both."
        ),
    )
    parser.add_argument(
        "--realtime",
        action="store_true",
        help="Also test real-time messages: Clock (0xF8), Start, Continue, Stop.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        metavar="SEC",
        help=f"Seconds to wait for each echoed message (default: {DEFAULT_TIMEOUT_S}).",
    )
    return parser


def main():
    try:
        import rtmidi
    except ImportError:
        util.install_rtmidi()
    finally:
        import rtmidi

    midiout = rtmidi.MidiOut()
    midiin = rtmidi.MidiIn()

    parser = argparser()
    ok = False
    args = None
    try:
        args = parser.parse_args()
        outport, inport, *_ = args.ports + [None, None]
        if outport is not None and inport is None:
            inport = outport

        outport = util.ensure_midi_port("output", midiout, outport)
        inport = util.ensure_midi_port("input ", midiin, inport)

        midiout.open_port(outport)
        midiin.open_port(inport)
        ok = True
    except Exception as e:
        util.note(f"ERROR: {e}")
    finally:
        if not ok:
            util.report_available_midi_ports("output", midiout)
            util.report_available_midi_ports("input", midiin)
            sys.exit(1)

    success = run_tests(midiout, midiin, args.realtime, args.timeout)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
