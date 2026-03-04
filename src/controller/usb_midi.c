

#include "usb_midi.h"
#include "RZA1/cpu_specific.h"
#include "RZA1/uart/sio_char.h"
#include "drivers/uart/uart.h"
#include "tusb.h"

// MIDI DIN-to-USB parser state — persists across usb_midi_task() invocations
static uint8_t s_midi_buffer[3]   = {0};
static int     s_midi_buffer_pos  = 0;
static uint8_t s_running_status   = 0;

// SysEx reassembly state for DIN→USB direction.
// SysEx body bytes are accumulated here three at a time; full triplets are
// dispatched as USB MIDI code 0x04 (SysEx continue) as they fill up, and the
// tail is sent with the appropriate end-of-SysEx code (0x05/0x06/0x07) when
// the 0xF7 terminator arrives.
static bool    s_in_sysex      = false;
static uint8_t s_sysex_buf[3]  = {0};
static int     s_sysex_buf_pos = 0;

void usb_midi_init(void) {
	// MIDI UART is already initialized in main (uartInit(UART_ITEM_MIDI, 31250))
	// No additional initialization needed
}

void usb_midi_task(void) {
	// Handle incoming MIDI from USB and send to UART
	if (tud_midi_available()) {
		uint8_t packet[4];
		while (tud_midi_packet_read(packet)) {
			// USB MIDI packet format: [cable_number_and_code_index, byte0, byte1, byte2]
			// Extract MIDI bytes and send via UART
			uint8_t code_index = packet[0] & 0x0F;

			// Determine number of MIDI bytes based on code index
			int num_bytes = 0;
			switch (code_index) {
			case 0x02: // Two-byte System Common (e.g., MTC, Song Select)
			case 0x06: // System Exclusive End (single byte)
			case 0x0C: // Program Change
			case 0x0D: // Channel Pressure
				num_bytes = 2;
				break;
			case 0x03: // Three-byte System Common (e.g., SPP)
			case 0x04: // System Exclusive Start/Continue
			case 0x07: // System Exclusive End (three bytes)
			case 0x08: // Note Off
			case 0x09: // Note On
			case 0x0A: // Poly Aftertouch
			case 0x0B: // Control Change
			case 0x0E: // Pitch Bend
				num_bytes = 3;
				break;
			case 0x05: // Single-byte System Common (e.g., Cable Select, Tune Request)
			case 0x0F: // Single Byte (e.g., Clock, Start, Stop)
				num_bytes = 1;
				break;
			default:
				continue; // Invalid code index
			}

			// Send MIDI bytes to UART
			for (int i = 0; i < num_bytes; i++) {
				bufferMIDIUart(packet[1 + i]);
			}
			uartFlushIfNotSending(UART_ITEM_MIDI);
		}
	}

	// Check for MIDI data from UART and send to USB
	char midi_byte;
	uint8_t midi_packet[4] = {0};

	while (uartGetChar(UART_ITEM_MIDI, &midi_byte)) {
		uint8_t byte = (uint8_t)midi_byte;

		// Real-time messages (0xF8–0xFF) may arrive inside a SysEx stream.
		// Forward immediately without disturbing any in-progress state.
		if (byte >= 0xF8) {
			midi_packet[0] = 0x0F; // Single Byte
			midi_packet[1] = byte;
			midi_packet[2] = 0;
			midi_packet[3] = 0;
			tud_midi_packet_write(midi_packet);
			continue;
		}

		// SysEx body byte while assembling a SysEx message.
		if (s_in_sysex && !(byte & 0x80)) {
			s_sysex_buf[s_sysex_buf_pos++] = byte;
			if (s_sysex_buf_pos == 3) {
				// Full triplet — emit as SysEx start/continue (code 0x04).
				midi_packet[0] = 0x04;
				midi_packet[1] = s_sysex_buf[0];
				midi_packet[2] = s_sysex_buf[1];
				midi_packet[3] = s_sysex_buf[2];
				tud_midi_packet_write(midi_packet);
				s_sysex_buf_pos = 0;
			}
			continue;
		}

		// SysEx end byte — flush the tail with the correct code index.
		if (s_in_sysex && byte == 0xF7) {
			s_in_sysex = false;
			switch (s_sysex_buf_pos) {
			case 0: // 0xF7 alone (or after an aligned triplet)
				midi_packet[0] = 0x05;
				midi_packet[1] = 0xF7;
				midi_packet[2] = 0;
				midi_packet[3] = 0;
				break;
			case 1: // one leftover byte then 0xF7
				midi_packet[0] = 0x06;
				midi_packet[1] = s_sysex_buf[0];
				midi_packet[2] = 0xF7;
				midi_packet[3] = 0;
				break;
			case 2: // two leftover bytes then 0xF7
				midi_packet[0] = 0x07;
				midi_packet[1] = s_sysex_buf[0];
				midi_packet[2] = s_sysex_buf[1];
				midi_packet[3] = 0xF7;
				break;
			default:
				break;
			}
			s_sysex_buf_pos = 0;
			tud_midi_packet_write(midi_packet);
			continue;
		}

		// Any non-real-time status byte while in SysEx aborts the stream.
		if (s_in_sysex && (byte & 0x80)) {
			s_in_sysex     = false;
			s_sysex_buf_pos = 0;
			s_running_status = 0;
			// Fall through to handle the new status byte normally.
		}

		if (byte & 0x80) {
			// Non-real-time status byte.
			if (byte == 0xF0) {
				// SysEx start — begin accumulation.
				s_in_sysex         = true;
				s_sysex_buf[0]     = 0xF0;
				s_sysex_buf_pos    = 1;
				s_running_status   = 0;
				s_midi_buffer_pos  = 0;
				continue;
			}
			if (byte >= 0xF0) {
				// Other system common (0xF1–0xF6) — clear running status.
				s_running_status = 0;
			}
			else {
				// Channel message — update running status.
				s_running_status = byte;
			}
			s_midi_buffer[0]  = byte;
			s_midi_buffer_pos = 1;
		}
		else {
			// Data byte (not in SysEx mode).
			if (s_running_status == 0) {
				continue; // No running status — discard.
			}
			if (s_midi_buffer_pos == 0) {
				// Running status — re-insert the status byte.
				s_midi_buffer[0]  = s_running_status;
				s_midi_buffer_pos = 1;
			}
			s_midi_buffer[s_midi_buffer_pos++] = byte;
		}

		// Check if we have a complete channel message.
		uint8_t status     = s_midi_buffer[0];
		int required_bytes = 0;
		uint8_t code_index = 0;

		if ((status & 0xF0) == 0x80 || (status & 0xF0) == 0x90 || (status & 0xF0) == 0xA0
		    || (status & 0xF0) == 0xB0 || (status & 0xF0) == 0xE0) {
			// Note Off, Note On, Poly Aftertouch, Control Change, Pitch Bend
			required_bytes = 3;
			code_index     = (status >> 4) & 0x0F;
		}
		else if ((status & 0xF0) == 0xC0 || (status & 0xF0) == 0xD0) {
			// Program Change, Channel Pressure
			required_bytes = 2;
			code_index     = (status >> 4) & 0x0F;
		}

		if (required_bytes > 0 && s_midi_buffer_pos >= required_bytes) {
			midi_packet[0] = code_index;
			midi_packet[1] = s_midi_buffer[0];
			midi_packet[2] = required_bytes >= 2 ? s_midi_buffer[1] : 0;
			midi_packet[3] = required_bytes >= 3 ? s_midi_buffer[2] : 0;
			tud_midi_packet_write(midi_packet);
			s_midi_buffer_pos = 0;
		}
	}
}

// TinyUSB MIDI callbacks
void tud_midi_rx_cb(uint8_t itf) {
	(void)itf;
	// MIDI data received from host - will be processed in usb_midi_task()
}
