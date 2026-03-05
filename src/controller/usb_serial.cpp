#include "usb_serial.h"
#include "controller_util.h"
#include "display_control.h"
#include "drivers/pic/pic.h"
#include "hardware_events.h"
#include "tusb.h"
#include <string.h>

extern "C" {
#include "RTT/SEGGER_RTT.h"
#include "RZA1/gpio/gpio.h"
}

// Message buffer for receiving
#define RX_BUFFER_SIZE 1024
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint32_t rx_buffer_pos = 0;

// Message buffer for sending
// 3-byte framing header + up to 255 bytes payload (usb_serial_send_error max) = 258 bytes;
// round up to 512 for headroom.
#define TX_BUFFER_SIZE 512
static uint8_t tx_buffer[TX_BUFFER_SIZE];

// Simple message framing: [length:2][type:1][data:N]
// Length includes type byte and data, but not the length field itself

static void send_message(uint8_t type, const uint8_t* data, uint16_t data_len) {
	if (!tud_cdc_connected()) {
		return;
	}

	uint16_t total_len = 1 + data_len; // type + data

	// Build message in tx_buffer
	tx_buffer[0] = total_len & 0xFF;
	tx_buffer[1] = (total_len >> 8) & 0xFF;
	tx_buffer[2] = type;

	if (data_len > 0 && data != NULL) {
		memcpy(&tx_buffer[3], data, data_len);
	}

	// Send via TinyUSB CDC
	tud_cdc_write(tx_buffer, 3 + data_len);
	tud_cdc_write_flush();
}

static void process_incoming_message(uint8_t type, const uint8_t* data, uint16_t data_len) {
	switch (type) {
	case MSG_TO_UPDATE_DISPLAY:
		if (data_len > 0) {
			display_update(data, data_len);
		}
		break;

	case MSG_TO_CLEAR_DISPLAY:
		display_clear();
		break;

	case MSG_TO_SET_PAD_RGB:
		if (data_len >= 5) {
			uint8_t col = data[0];
			uint8_t row = data[1];
			uint8_t r = data[2];
			uint8_t g = data[3];
			uint8_t b = data[4];
			pad_led_set_rgb(col, row, r, g, b);
		}
		break;

	case MSG_TO_CLEAR_ALL_PADS:
		pad_led_clear_all();
		break;

	case MSG_TO_SET_ALL_PADS:
		if (data_len >= 18 * 8 * 3) {
			pad_led_set_all(data, data_len);
		}
		break;

	case MSG_TO_SET_KNOB_INDICATOR:
		// data: [which(0|1), brightness0, brightness1, brightness2, brightness3]
		if (data_len >= 5) {
			uint8_t which = data[0] & 1;
			std::array<uint8_t, kNumGoldKnobIndicatorLEDs> levels = {data[1], data[2], data[3], data[4]};
			PIC::setGoldKnobIndicator(which, levels);
			PIC::flush();
		}
		break;

	case MSG_TO_SET_SYNCED_LED:
		// Tempo external-synced indicator — dedicated GPIO, not a PIC LED.
		// SYNCED_LED = {port:6, pin:7} (from definitions_cxx.hpp)
		if (data_len >= 1) {
			setOutputState(6, 7, data[0] != 0);
		}
		break;

	case MSG_TO_SET_LED:
		if (data_len >= 2) {
			uint8_t led_index = data[0];
			bool on = data[1] != 0;
			hardware_led_set(led_index, on);
		}
		break;

	case MSG_TO_CLEAR_ALL_LEDS:
		// Turn off all 36 button indicator LEDs
		for (uint8_t i = 0; i < 36; ++i) {
			hardware_led_set(i, false);
		}
		// Clear both gold knob indicator bars
		{
			std::array<uint8_t, kNumGoldKnobIndicatorLEDs> off = {0, 0, 0, 0};
			PIC::setGoldKnobIndicator(0, off);
			PIC::setGoldKnobIndicator(1, off);
		}
		// Clear the synced GPIO LED
		setOutputState(6, 7, false);
		PIC::flush();
		break;

	case MSG_TO_SET_CV:
		if (data_len >= 3) {
			uint8_t channel = data[0];
			uint16_t value = (data[1] << 8) | data[2];
			cv_set(channel, value);
		}
		break;

	case MSG_TO_SET_GATE:
		if (data_len >= 2) {
			uint8_t channel = data[0];
			bool on = data[1] != 0;
			gate_set(channel, on);
		}
		break;

	case MSG_TO_GET_VERSION:
		usb_serial_send_version();
		break;

	case MSG_TO_PING:
		usb_serial_send_pong();
		break;

	default:
		// Unknown message type
		break;
	}
}

void usb_serial_init(void) {
	// USB initialization is done in main
	rx_buffer_pos = 0;
}

void usb_serial_task(void) {
	if (!tud_cdc_connected()) {
		return;
	}

	// ── Dispatch queued hardware events to host ───────────────────────
	// This must run every call regardless of incoming USB data.
	{
		HardwareEvent ev;
		while (hardware_events_pop(&ev)) {
			switch (ev.type) {
			case EVENT_TYPE_PAD_PRESS:
			case EVENT_TYPE_PAD_RELEASE: {
				// Decode using Pad::Pad(uint8_t) formula from pad.cpp:
				//   y = id / 9
				//   x = (id - y*9) * 2;  if y >= kDisplayHeight(8): y -= 8, x += 1
				uint8_t y = ev.id / 9;
				uint8_t x = (ev.id - y * 9) * 2;
				if (y >= 8) {
					y -= 8;
					x += 1;
				}
				if (ev.type == EVENT_TYPE_PAD_PRESS) {
					usb_serial_send_pad_pressed(x, y);
				}
				else {
					usb_serial_send_pad_released(x, y);
				}
				break;
			}
			case EVENT_TYPE_BUTTON_PRESS:
				usb_serial_send_button_pressed(ev.id);
				break;
			case EVENT_TYPE_BUTTON_RELEASE:
				usb_serial_send_button_released(ev.id);
				break;
			case EVENT_TYPE_ENCODER:
				usb_serial_send_encoder_rotated(ev.id, (int8_t)(int16_t)ev.value);
				break;
			default:
				break;
			}
		}
	}

	// Read available data
	uint32_t available = tud_cdc_available();
	if (available == 0) {
		return;
	}

	// Read into buffer
	uint32_t space = RX_BUFFER_SIZE - rx_buffer_pos;
	if (space > 0) {
		uint32_t to_read = (available < space) ? available : space;
		uint32_t read = tud_cdc_read(&rx_buffer[rx_buffer_pos], to_read);
		rx_buffer_pos += read;
	}

	// Process complete messages
	while (rx_buffer_pos >= 3) { // Minimum: 2-byte length + 1-byte type
		uint16_t msg_len = rx_buffer[0] | (rx_buffer[1] << 8);
		uint16_t total_len = 2 + msg_len; // length field + message

		if (rx_buffer_pos >= total_len) {
			// Complete message available
			uint8_t type = rx_buffer[2];
			uint16_t data_len = msg_len - 1; // Subtract type byte
			const uint8_t* data = (data_len > 0) ? &rx_buffer[3] : NULL;

			process_incoming_message(type, data, data_len);

			// Remove processed message from buffer
			if (rx_buffer_pos > total_len) {
				memmove(rx_buffer, &rx_buffer[total_len], rx_buffer_pos - total_len);
				rx_buffer_pos -= total_len;
			}
			else {
				rx_buffer_pos = 0;
			}
		}
		else {
			// Incomplete message, wait for more data
			break;
		}
	}
	// pad_led_flush_dirty() is called from the main loop on every iteration so that
	// remaining dirty pairs drain as soon as UART buffer space opens up, regardless
	// of whether any USB data arrived this call.
}

bool usb_serial_is_connected(void) {
	return tud_cdc_connected();
}

// Send message implementations
void usb_serial_send_pad_pressed(uint8_t col, uint8_t row) {
	uint8_t data[2] = {col, row};
	send_message(MSG_FROM_PAD_PRESSED, data, 2);
}

void usb_serial_send_pad_released(uint8_t col, uint8_t row) {
	uint8_t data[2] = {col, row};
	send_message(MSG_FROM_PAD_RELEASED, data, 2);
}

void usb_serial_send_button_pressed(uint8_t button_id) {
	send_message(MSG_FROM_BUTTON_PRESSED, &button_id, 1);
}

void usb_serial_send_button_released(uint8_t button_id) {
	send_message(MSG_FROM_BUTTON_RELEASED, &button_id, 1);
}

void usb_serial_send_encoder_rotated(uint8_t encoder_id, int8_t delta) {
	uint8_t data[2] = {encoder_id, (uint8_t)delta};
	send_message(MSG_FROM_ENCODER_ROTATED, data, 2);
}

void usb_serial_send_version(void) {
	uint8_t data[3] = {USB_SERIAL_VERSION_MAJOR, USB_SERIAL_VERSION_MINOR, USB_SERIAL_VERSION_PATCH};
	send_message(MSG_FROM_VERSION, data, 3);
}

void usb_serial_send_pong(void) {
	send_message(MSG_FROM_PONG, NULL, 0);
}

void usb_serial_send_ready(void) {
	send_message(MSG_FROM_READY, NULL, 0);
}

void usb_serial_send_error(const char* error_msg) {
	uint16_t len = strlen(error_msg);
	if (len > 255)
		len = 255;
	send_message(MSG_FROM_ERROR, (const uint8_t*)error_msg, len);
}

// TinyUSB callbacks for debugging
extern "C" {

// Invoked when device is mounted
void tud_mount_cb(void) {
	CDBG_STR("*** USB MOUNTED ***\n");
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
	CDBG_STR("*** USB UNMOUNTED ***\n");
}

// Invoked when USB bus is suspended
void tud_suspend_cb(bool remote_wakeup_en) {
	(void)remote_wakeup_en;
	CDBG_STR("*** USB SUSPENDED ***\n");
}

// Invoked when USB bus is resumed
void tud_resume_cb(void) {
	CDBG_STR("*** USB RESUMED ***\n");
}

} // extern "C"
