

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Deluge USB Serial Protocol  (version 1.0)
// ---------------------------------------------------------------------------
//
// Transport
// ---------
// USB CDC class (virtual serial port).  The host opens the Deluge CDC
// interface and exchanges length-prefixed binary messages.
//
// Framing
// -------
// Every message has the following wire layout:
//
//   Offset  Size  Field
//   ------  ----  -----
//     0       2   length  (uint16_t, little-endian)
//                 = sizeof(type) + sizeof(data) = 1 + N
//     2       1   type    (MessageToDelugeType or MessageFromDelugeType)
//     3       N   data    (payload, may be empty)
//
// The length field does NOT include itself; minimum message size on the
// wire is 3 bytes (length=1, type, no payload).
//
// Messages FROM Deluge TO Host (device → host)
// --------------------------------------------
//   MSG_FROM_PAD_PRESSED    [col:1][row:1]
//     col: 0-17 (columns 0-15 = main grid, 16-17 = sidebar)
//     row: 0-7
//
//   MSG_FROM_PAD_RELEASED   [col:1][row:1]   (same layout as PRESSED)
//
//   MSG_FROM_BUTTON_PRESSED  [button_id:1]
//     button_id: raw PIC button code 144–179.  Refer to the PIC firmware
//     source for the mapping of code → physical button name.
//
//   MSG_FROM_BUTTON_RELEASED [button_id:1]   (same layout as PRESSED)
//
//   MSG_FROM_ENCODER_ROTATED [encoder_id:1][delta:1]
//     encoder_id: 0=SCROLL_X, 1=TEMPO, 2=MOD_0, 3=MOD_1, 4=SCROLL_Y, 5=SELECT
//     delta: signed byte (int8_t); positive = clockwise
//
//   NOTE — encoder button presses are reported as MSG_FROM_BUTTON_PRESSED /
//   MSG_FROM_BUTTON_RELEASED with the following button_ids (144 + x + y*9):
//     144 = SCROLL_Y push  (yEncButtonCoord     {0,0})
//     153 = SCROLL_X push  (xEncButtonCoord     {0,1})
//     157 = TEMPO push     (tempoEncButtonCoord  {4,1})
//     162 = MOD_0 push     (modEncoder0ButtonCoord {0,2})
//     171 = MOD_1 push     (modEncoder1ButtonCoord {0,3})
//     175 = SELECT push    (selectEncButtonCoord {4,3})
//
//   MSG_FROM_VERSION         [major:1][minor:1][patch:1]
//     Sent in response to MSG_TO_GET_VERSION.
//
//   MSG_FROM_PONG            (no payload)
//     Sent in response to MSG_TO_PING.
//
//   MSG_FROM_READY           (no payload)
//     Sent once when the controller completes initialisation and is ready
//     to accept commands.
//
//   MSG_FROM_ERROR           [message:N]   (N ≤ 255, UTF-8 text)
//     Sent when an unrecoverable protocol error is detected.
//
// Messages TO Deluge FROM Host (host → device)
// --------------------------------------------
//   MSG_TO_UPDATE_DISPLAY    [pixels:N]
//     N = OLED_MAIN_WIDTH_PIXELS × (OLED_MAIN_HEIGHT_PIXELS/8) bytes
//     = 128 × 8 = 1024 bytes.
//     Pixel layout: column-major, 1 bit per pixel, LSB = top row.
//
//   MSG_TO_CLEAR_DISPLAY     (no payload)
//
//   MSG_TO_SET_PAD_RGB       [col:1][row:1][r:1][g:1][b:1]
//     Sets a single pad LED.  col 0-17, row 0-7.
//
//   MSG_TO_CLEAR_ALL_PADS    (no payload)
//     Turns off all pad LEDs.
//
//   MSG_TO_SET_ALL_PADS      [data: 18×8×3 bytes = 432 bytes]
//     Bulk RGB update.  Layout: col-major, R/G/B per cell.
//     data[col][row] = {R, G, B}  at offset (col*8 + row)*3.
//
//   MSG_TO_SET_LED           [led_index:1][on:1]
//     Controls an indicator LED (0–35).  on: 0=off, 1=on.
//
//   MSG_TO_SET_KNOB_INDICATOR  [which:1][b0:1][b1:1][b2:1][b3:1]
//     Sets the four indicator LEDs around one of the two gold knobs.
//     which: 0 or 1.  b0–b3: brightness values for each LED segment.
//
//   MSG_TO_SET_SYNCED_LED    [on:1]
//     Controls the external-sync LED on the rear panel.  on: 0=off, 1=on.
//
//   MSG_TO_SET_CV            [channel:1][value_hi:1][value_lo:1]
//     Sets a CV output voltage.  channel: 0–3.
//     value = (value_hi << 8) | value_lo  →  0–65535 linear across 0–10 V.
//
//   MSG_TO_SET_GATE          [channel:1][on:1]
//     Sets a gate output.  channel: 0–3.  on: 0=off, 1=on (+5 V).
//
//   MSG_TO_GET_VERSION       (no payload)  →  triggers MSG_FROM_VERSION
//
//   MSG_TO_PING              (no payload)  →  triggers MSG_FROM_PONG
// ---------------------------------------------------------------------------

// Protocol version
#define USB_SERIAL_VERSION_MAJOR 1
#define USB_SERIAL_VERSION_MINOR 0
#define USB_SERIAL_VERSION_PATCH 0

// Message types FROM Deluge TO Host
typedef enum {
	MSG_FROM_PAD_PRESSED = 0x01,
	MSG_FROM_PAD_RELEASED = 0x02,
	MSG_FROM_BUTTON_PRESSED = 0x03,  // includes encoder push buttons — see ID table above
	MSG_FROM_BUTTON_RELEASED = 0x04, // includes encoder push buttons — see ID table above
	MSG_FROM_ENCODER_ROTATED = 0x05,
	MSG_FROM_VERSION = 0x10,
	MSG_FROM_PONG = 0x11,
	MSG_FROM_READY = 0x12,
	MSG_FROM_ERROR = 0x13,
} MessageFromDelugeType;

// Message types TO Deluge FROM Host
typedef enum {
	MSG_TO_UPDATE_DISPLAY = 0x20,
	MSG_TO_CLEAR_DISPLAY = 0x21,
	MSG_TO_SET_PAD_RGB = 0x22,
	MSG_TO_CLEAR_ALL_PADS = 0x23,
	MSG_TO_SET_LED = 0x24,
	MSG_TO_SET_CV = 0x25,
	MSG_TO_SET_GATE = 0x26,
	MSG_TO_SET_ALL_PADS = 0x27,
	MSG_TO_SET_KNOB_INDICATOR = 0x28, // data: [which(0|1), b0, b1, b2, b3]
	MSG_TO_SET_SYNCED_LED = 0x29,     // data: [on(0|1)]
	MSG_TO_GET_VERSION = 0x30,
	MSG_TO_PING = 0x31,
} MessageToDelugeType;

// Initialize USB serial protocol
void usb_serial_init(void);

// Process USB serial tasks (send/receive messages)
void usb_serial_task(void);

// Check if USB is connected and ready
bool usb_serial_is_connected(void);

// Send messages FROM Deluge TO Host
void usb_serial_send_pad_pressed(uint8_t col, uint8_t row);
void usb_serial_send_pad_released(uint8_t col, uint8_t row);
void usb_serial_send_button_pressed(uint8_t button_id);
void usb_serial_send_button_released(uint8_t button_id);
void usb_serial_send_encoder_rotated(uint8_t encoder_id, int8_t delta);
void usb_serial_send_version(void);
void usb_serial_send_pong(void);
void usb_serial_send_ready(void);
void usb_serial_send_error(const char* error_msg);

#ifdef __cplusplus
}
#endif
