

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize USB audio subsystem
void usb_audio_init(void);

// USB audio task - call periodically to handle audio streaming
void usb_audio_task(void);

// Check if USB audio is currently streaming
bool usb_audio_is_streaming(void);

// Enable/disable the speaker based on audio streaming state
void usb_audio_update_speaker_state(void);

#ifdef __cplusplus
}
#endif
