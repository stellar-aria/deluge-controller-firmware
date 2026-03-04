// DelugeASIO — USB device constants
// Must match the descriptors in src/controller/usb_descriptors.c/.h

#pragma once

#include <cstdint>

namespace deluge {

// USB identifiers
constexpr uint16_t kVendorId  = 0x16D0; // MCS Electronics (open-source)
constexpr uint16_t kProductId = 0x0EDA; // Deluge Controller

// Interface numbers (must match ITF_NUM_* enum in usb_descriptors.h)
constexpr uint8_t kItfAudioControl     = 0;
constexpr uint8_t kItfAudioStreamSpk   = 1;
constexpr uint8_t kItfAudioStreamMic   = 2;

// UAC2 entity IDs (must match UAC2_ENTITY_* in usb_descriptors.h)
constexpr uint8_t kClockSourceEntity = 0x04;

// Endpoint addresses
constexpr uint8_t kEpAudioOut = 0x03; // Speaker OUT (PC → Deluge), ISO
constexpr uint8_t kEpAudioIn  = 0x83; // Mic IN (Deluge → PC), ISO, implicit feedback

// Audio parameters (fixed by codec crystal)
constexpr uint32_t kSampleRate     = 44100;
constexpr uint32_t kNumChannels    = 2;  // Stereo

// Alternate settings
constexpr uint8_t kAltIdle   = 0; // Zero-bandwidth
constexpr uint8_t kAlt16Bit  = 1; // 16-bit PCM
constexpr uint8_t kAlt24Bit  = 2; // 24-bit PCM

// Max ISO packet size per alt (from TUD_AUDIO_EP_SIZE for high-speed, 44100 Hz, stereo)
// HS microframe = 125 µs, so 44100/8000 = 5.5125 samples/µframe
// 16-bit: ceil(5.5125) * 2ch * 2bytes = 6 * 4 = 24 bytes  (+ 1 extra sample: 28)
// 24-bit: ceil(5.5125) * 2ch * 3bytes = 6 * 6 = 36 bytes  (+ 1 extra: 42)
// TUD_AUDIO_EP_SIZE adds one extra sample for async tolerance
constexpr uint32_t kMaxPacketSize16 = 28;
constexpr uint32_t kMaxPacketSize24 = 42;

// Nominal samples per microframe at 44100 Hz (HS)
// 44100 / 8000 = 5.5125 — alternates between 5 and 6 samples
constexpr double kSamplesPerMicroframe = 44100.0 / 8000.0;

// ISO transfer parameters
constexpr uint32_t kMicroframesPerTransfer = 8;  // 8 µframes = 1 ms
constexpr uint32_t kPendingTransfers       = 4;  // Double-double buffering

} // namespace deluge
