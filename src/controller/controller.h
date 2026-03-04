#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Main controller entry point
int32_t controller_main(void);

// Stub for MIDI/gate timer callback (not used in controller mode)
void midiAndGateTimerGoneOff(void);

#ifdef __cplusplus
}
#endif
