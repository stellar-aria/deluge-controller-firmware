#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Event types
typedef enum {
	EVENT_TYPE_BUTTON_PRESS   = 0x01,
	EVENT_TYPE_BUTTON_RELEASE = 0x02,
	EVENT_TYPE_ENCODER        = 0x03,
	EVENT_TYPE_PAD_PRESS      = 0x04,
	EVENT_TYPE_PAD_RELEASE    = 0x05,
	EVENT_TYPE_MIDI_IN        = 0x10, // MIDI input from DIN ports
} EventType;

// Hardware event structure
typedef struct {
	EventType type;
	uint8_t id;         // Button/encoder/pad identifier or MIDI status byte
	uint16_t value;     // Value (encoder delta, velocity, MIDI data bytes, etc.)
	uint32_t timestamp; // Timestamp in milliseconds
} HardwareEvent;

// MIDI event helper function
void hardware_events_push_midi_in(uint8_t status, uint8_t data1, uint8_t data2);

// Event queue
#define EVENT_QUEUE_SIZE 64

// Initialize hardware event system
void hardware_events_init(void);

// Scan hardware for events
void hardware_events_scan(void);

// Get next event from queue (returns false if queue empty)
bool hardware_events_pop(HardwareEvent* event);

// Get number of events in queue
uint32_t hardware_events_count(void);

#ifdef __cplusplus
}
#endif
