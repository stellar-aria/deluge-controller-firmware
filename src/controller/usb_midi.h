#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Initialize USB MIDI subsystem
void usb_midi_init(void);

// USB MIDI task - call periodically to handle MIDI I/O
void usb_midi_task(void);

#ifdef __cplusplus
}
#endif
