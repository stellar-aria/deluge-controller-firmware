

#include "hardware_events.h"
#include "drivers/pic/pic.h"

extern "C" {
#include "RTT/SEGGER_RTT.h"
#include "RZA1/cpu_specific.h"
#include "RZA1/gpio/gpio.h"
#include "RZA1/mtu/mtu.h"
#include "drivers/oled/oled_low_level.h" // For oledWaitingForMessage and callback
#include "RZA1/system/r_typedefs.h"
#include "RZA1/uart/sio_char.h"
#include "definitions.h"
#include "drivers/uart/uart.h"
#include <string.h>
}

// Event queue for hardware events
static HardwareEvent event_queue[EVENT_QUEUE_SIZE];
static volatile uint32_t queue_head = 0;
static volatile uint32_t queue_tail = 0;

// PIC32 response codes (matches PIC firmware)
#define PIC_RESPONSE_NO_PRESSES 254
#define PIC_RESPONSE_PAD_OFF 252
#define PIC_PAD_BUTTON_MESSAGES_END 180

// ── Quadrature encoder decoder ───────────────────────────────────────────────
// Pin assignments match Encoder::init() in src/deluge/hid/encoders.cpp.
// Decode logic mirrors Encoder::read() in src/deluge/hid/encoder.cpp.
#define NUM_ENCODERS 6

struct EncoderState {
	uint8_t port_a, pin_a, port_b, pin_b;
	bool    pin_a_last_switch, pin_b_last_switch;
	bool    pin_a_last_read,   pin_b_last_read;
	int8_t  enc_pos;
	int8_t  enc_last_change;
	int8_t  detent_pos; // accumulated detent clicks to report
};

static EncoderState encoders[NUM_ENCODERS] = {
	// id  port_a  pin_a  port_b  pin_b
	{1, 11, 1, 12, true, true, true, true, 0, 0, 0}, // 0 SCROLL_X
	{1,  7, 1,  6, true, true, true, true, 0, 0, 0}, // 1 TEMPO
	{1,  0, 1, 15, true, true, true, true, 0, 0, 0}, // 2 MOD_0
	{1,  5, 1,  4, true, true, true, true, 0, 0, 0}, // 3 MOD_1
	{1,  8, 1, 10, true, true, true, true, 0, 0, 0}, // 4 SCROLL_Y
	{1,  2, 1,  3, true, true, true, true, 0, 0, 0}, // 5 SELECT
};

static void encoders_init(void) {
	for (int i = 0; i < NUM_ENCODERS; i++) {
		setPinAsInput(encoders[i].port_a, encoders[i].pin_a);
		setPinAsInput(encoders[i].port_b, encoders[i].pin_b);
	}
}

static void encoders_read(void) {
	for (int i = 0; i < NUM_ENCODERS; i++) {
		EncoderState& e = encoders[i];
		bool a = (bool)readInput(e.port_a, e.pin_a);
		bool b = (bool)readInput(e.port_b, e.pin_b);

		if (a != e.pin_a_last_switch && b != e.pin_b_last_switch) {
			int8_t change = 0;
			if (e.pin_a_last_read != e.pin_a_last_switch) {
				change = (e.pin_a_last_switch == e.pin_b_last_switch) ? -1 : 1;
				e.pin_a_last_switch = a;
			} else if (e.pin_b_last_read != e.pin_b_last_switch) {
				change = (e.pin_a_last_switch == e.pin_b_last_switch) ? 1 : -1;
				e.pin_b_last_switch = b;
			} else {
				change = (e.enc_last_change >= 0) ? 2 : -2;
				e.pin_a_last_switch = a;
				e.pin_b_last_switch = b;
			}
			if (change != 0) {
				e.enc_pos += change;
				while (e.enc_pos >  2) { e.enc_pos -= 4; e.detent_pos += 1; }
				while (e.enc_pos < -2) { e.enc_pos += 4; e.detent_pos -= 1; }
				e.enc_last_change = change;
			}
		}
		e.pin_a_last_read = a;
		e.pin_b_last_read = b;
	}
}

static bool push_event(const HardwareEvent* event) {
	uint32_t next_head = (queue_head + 1) % EVENT_QUEUE_SIZE;
	if (next_head == queue_tail) {
		SEGGER_RTT_WriteString(0, "[EVT] queue overflow — event dropped\n");
		return false;
	}
	memcpy(&event_queue[queue_head], event, sizeof(HardwareEvent));
	// DMB: ensure the slot write is visible before the index advance so that
	// any concurrent reader on the same core cannot observe a stale slot.
	__asm volatile("dmb" ::: "memory");
	queue_head = next_head;
	return true;
}

void hardware_events_init(void) {
	queue_head = 0;
	queue_tail = 0;

	// Initialise encoder GPIO pins
	encoders_init();

	// Request PIC32 to resend all button states using the binary-safe PIC API
	PIC::resendButtonStates();
	PIC::flush();
}

void hardware_events_scan(void) {
	// PIC protocol (matches deluge.cpp inputRoutine / Pad::isPad boundary):
	//
	//   value < 144  (= kDisplayHeight*2*9)  : pad   — SINGLE byte, press/release
	//                                           determined by preceding NEXT_PAD_OFF (252)
	//   144 <= value < 180                   : button — SINGLE byte, same press/release logic
	//   180 <= value < 248                   : encoder — TWO bytes: cmd + signed delta
	//   248 / 249                            : OLED select / deselect handshake (skip if not waiting)
	//   250 / 251 / 253 / 255                : undefined — skip
	//   252  NEXT_PAD_OFF                    : prefix — next pad/button is a release
	//   254  NO_PRESSES_HAPPENING            : no-op

	static constexpr uint8_t kIsPadMax = 8 * 2 * 9; // 144  — kDisplayHeight * 2 * 9

	// Persistent state for the two-byte encoder sequence that may straddle calls.
	static bool    next_is_off       = false;
	static bool    awaiting_encoder2 = false;
	static uint8_t pending_encoder   = 0;

	// Poll encoder GPIO pins and queue any detent events
	encoders_read();
	for (int i = 0; i < NUM_ENCODERS; i++) {
		if (encoders[i].detent_pos != 0) {
			HardwareEvent ev;
			ev.timestamp  = *TCNT[TIMER_SYSTEM_SLOW];
			ev.type       = EVENT_TYPE_ENCODER;
			ev.id         = (uint8_t)i;
			ev.value      = (uint16_t)(int16_t)(int8_t)encoders[i].detent_pos;
			encoders[i].detent_pos = 0;
			push_event(&ev);
		}
	}

	char value_char;

	// Resume an encoder sequence whose delta byte hadn't arrived yet.
	if (awaiting_encoder2) {
		if (uartGetChar(UART_ITEM_PIC, &value_char) == 0) {
			return; // Still not here — retry next call
		}
		awaiting_encoder2 = false;
		HardwareEvent event;
		event.timestamp = *TCNT[TIMER_SYSTEM_SLOW];
		event.type  = EVENT_TYPE_ENCODER;
		event.id    = pending_encoder - 180;
		event.value = (uint16_t)(int16_t)(int8_t)value_char;
		push_event(&event);
	}

	while (uartGetChar(UART_ITEM_PIC, &value_char) != 0) {
		uint8_t value = (uint8_t)value_char;
		HardwareEvent event;
		event.timestamp = *TCNT[TIMER_SYSTEM_SLOW];

		// OLED select/deselect handshake
		if (oledWaitingForMessage != 256 && value == (uint8_t)oledWaitingForMessage) {
			oledLowLevelTimerCallback();
			continue;
		}

		// "No presses happening" message
		if (value == PIC_RESPONSE_NO_PRESSES) {
			next_is_off = false;
			continue;
		}

		// NEXT_PAD_OFF prefix: the following pad or button is a release
		if (value == PIC_RESPONSE_PAD_OFF) {
			next_is_off = true;
			continue;
		}

		if (value < PIC_PAD_BUTTON_MESSAGES_END) { // value < 180
			if (value < kIsPadMax) {
				// Pad (0–143) — single byte, no follow-up
				event.id   = value;
				event.type = next_is_off ? EVENT_TYPE_PAD_RELEASE : EVENT_TYPE_PAD_PRESS;
				event.value = 0;
				push_event(&event);
			}
			else {
				// Button (144–179) — single byte
				event.type  = next_is_off ? EVENT_TYPE_BUTTON_RELEASE : EVENT_TYPE_BUTTON_PRESS;
				event.id    = value;
				event.value = 0;
				push_event(&event);
			}
			next_is_off = false;
		}
		else {
			// Encoder (180–247) — followed by a signed delta byte.
			//
			// Bytes in the range 248–255 are reserved or OLED handshake codes
			// (248 = OLED select, 249 = OLED deselect, 252 = PAD_OFF (handled
			// above), 254 = NO_PRESSES (handled above)).  The OLED-handshake
			// check near the top of this loop only fires when oledWaitingForMessage
			// is active; stray 248/249 bytes at other times would otherwise fall
			// through here and be misinterpreted as encoder messages with IDs 68
			// and 69 (and similarly 250–255 → IDs 70–75).  Skip them explicitly.
			if (value >= 248) {
				continue;
			}
			char next_char;
			if (uartGetChar(UART_ITEM_PIC, &next_char) == 0) {
				// Delta byte not yet in buffer — park and retry next call
				awaiting_encoder2 = true;
				pending_encoder   = value;
				return;
			}
			event.type  = EVENT_TYPE_ENCODER;
			event.id    = value - 180;
			event.value = (uint16_t)(int16_t)(int8_t)next_char;
			push_event(&event);
		}
	}
}

bool hardware_events_pop(HardwareEvent* event) {
	if (queue_head == queue_tail) {
		return false;
	}
	// DMB: ensure we read the slot only after the producer's index advance is visible.
	__asm volatile("dmb" ::: "memory");
	memcpy(event, &event_queue[queue_tail], sizeof(HardwareEvent));
	queue_tail = (queue_tail + 1) % EVENT_QUEUE_SIZE;
	return true;
}

uint32_t hardware_events_count(void) {
	if (queue_head >= queue_tail) {
		return queue_head - queue_tail;
	}
	return EVENT_QUEUE_SIZE - queue_tail + queue_head;
}

void hardware_events_push_midi_in(uint8_t status, uint8_t data1, uint8_t data2) {
	HardwareEvent event;
	event.type = EVENT_TYPE_MIDI_IN;
	event.id = status;
	event.value = (data1 << 8) | data2;
	event.timestamp = *TCNT[TIMER_SYSTEM_SLOW];
	push_event(&event);
}
