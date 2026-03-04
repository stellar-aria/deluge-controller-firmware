#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "usb_descriptors.h"
#include <stdarg.h>
#include <stdio.h>

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

// RZA1L MCU
#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU OPT_MCU_RZA1X
#endif

// RUSB1 specific configuration (required by TinyUSB RZA1 driver)
// Clock source: 0 = USB_X1 48MHz crystal, 1 = EXTAL 12MHz
#ifndef RUSB1_CLOCK_SOURCE
#define RUSB1_CLOCK_SOURCE 0 // Deluge uses USB_X1 48MHz
#endif

// Wait cycles for register access (must result in > 67ns wait time)
// With P1 bus @ 33.33MHz (30ns/cycle), need ≥67ns / 30ns = 2.23 cycles minimum
// Using 5 wait cycles (7 total access cycles) = 210ns >> 67ns (matches Deluge config)
#ifndef RUSB1_WAIT_CYCLES
#define RUSB1_WAIT_CYCLES 5
#endif

// No RTOS
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_NONE
#endif

// Debug level (0 = no debug, 1 = errors only, 2 = warnings, 3 = verbose)
#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

// Route TinyUSB debug output to Segger RTT channel 0
// Declare RTT function directly to avoid include path issues with TinyUSB lib builds
extern unsigned SEGGER_RTT_Write(unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
#define CFG_TUSB_DEBUG_PRINTF rtt_printf
static inline int rtt_printf(const char* format, ...) {
	char buf[256];
	va_list args;
	va_start(args, format);
	int len = vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
	SEGGER_RTT_Write(0, buf, (unsigned)len);
	return len;
}

// Enable Device stack
#define CFG_TUD_ENABLED 1

// RHPort configuration
#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE
#define CFG_TUSB_RHPORT1_MODE 0

// RHPort number used for device
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT 0
#endif

// RHPort max operational speed - High-Speed for UAC2 bandwidth
#ifndef BOARD_TUD_MAX_SPEED
#define BOARD_TUD_MAX_SPEED OPT_MODE_HIGH_SPEED
#endif

#define CFG_TUD_MAX_SPEED BOARD_TUD_MAX_SPEED

/* USB DMA on some MCUs can only access a specific SRAM region with restriction on alignment.
 * Tinyusb use follows macros to declare transferring memory so that they can be put
 * into those specific section.
 * e.g
 * - CFG_TUSB_MEM SECTION : __attribute__ (( section(".usb_ram") ))
 * - CFG_TUSB_MEM_ALIGN   : __attribute__ ((aligned(4)))
 */
#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE 64
#endif

//------------- CLASS -------------//
#define CFG_TUD_CDC 1
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 1
#define CFG_TUD_AUDIO 1
#define CFG_TUD_VENDOR 0

// CDC FIFO size of TX and RX
#define CFG_TUD_CDC_RX_BUFSIZE 512
#define CFG_TUD_CDC_TX_BUFSIZE 512

// CDC Endpoint transfer buffer size (512 bytes for High-Speed bulk)
#define CFG_TUD_CDC_EP_BUFSIZE 512

// MIDI FIFO size of TX and RX
#define CFG_TUD_MIDI_RX_BUFSIZE 512
#define CFG_TUD_MIDI_TX_BUFSIZE 512

//--------------------------------------------------------------------
// AUDIO CLASS DRIVER CONFIGURATION
//--------------------------------------------------------------------

#if USB_AUDIO_SPEAKER_ONLY
// Speaker-only: explicit feedback EP, no microphone

// Audio descriptor length - speaker with feedback EP
#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN TUD_AUDIO_SPEAKER_STEREO_FB_DESC_LEN

// Number of Standard AS Interface Descriptors (speaker only)
#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT 1

#else
// Headset: speaker + microphone with implicit feedback

// Audio descriptor length - speaker + microphone
#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN TUD_AUDIO_HEADSET_STEREO_DESC_LEN

// Number of Standard AS Interface Descriptors (speaker + microphone)
#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT 2

#endif // USB_AUDIO_SPEAKER_ONLY

// Size of control request buffer
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ 64

// Audio format type I specifications - 44.1kHz stereo, supports 16-bit and 24-bit
#define CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE 44100

// Speaker (RX from host perspective, OUT endpoint)
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX 2         // Stereo output (PC to Deluge)
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX 3 // Max: 24-bit = 3 bytes per sample
#define CFG_TUD_AUDIO_FUNC_1_RESOLUTION_RX 24        // Max: 24-bit resolution

#if !USB_AUDIO_SPEAKER_ONLY
// Microphone (TX from host perspective, IN endpoint) - only in headset mode
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX 2         // Stereo input (Deluge to PC)
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX 3 // Max: 24-bit = 3 bytes per sample
#define CFG_TUD_AUDIO_FUNC_1_RESOLUTION_TX 24        // Max: 24-bit resolution
#endif

// Enable endpoints
#define CFG_TUD_AUDIO_ENABLE_EP_OUT 1 // Speaker output (PC to Deluge) - always enabled

#if USB_AUDIO_SPEAKER_ONLY
// Speaker-only: no mic, use explicit feedback EP instead
#define CFG_TUD_AUDIO_ENABLE_EP_IN 0
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP 1
#else
// Headset: mic IN serves as implicit feedback source, no explicit feedback EP
#define CFG_TUD_AUDIO_ENABLE_EP_IN 1
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP 0
#endif

// Calculate EP size for 44.1kHz stereo at each bit depth
// TUD_AUDIO_EP_SIZE auto-adjusts for HS (8 microframes/ms) vs FS (1 frame/ms)
#define CFG_TUD_AUDIO_FUNC_1_EP_SZ_OUT_16                                                                              \
	TUD_AUDIO_EP_SIZE(TUD_OPT_HIGH_SPEED, CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, 2, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)
#define CFG_TUD_AUDIO_FUNC_1_EP_SZ_OUT_24                                                                              \
	TUD_AUDIO_EP_SIZE(TUD_OPT_HIGH_SPEED, CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, 3, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)

#if CFG_TUD_AUDIO_ENABLE_EP_IN
#define CFG_TUD_AUDIO_FUNC_1_EP_SZ_IN_16                                                                               \
	TUD_AUDIO_EP_SIZE(TUD_OPT_HIGH_SPEED, CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, 2, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)
#define CFG_TUD_AUDIO_FUNC_1_EP_SZ_IN_24                                                                               \
	TUD_AUDIO_EP_SIZE(TUD_OPT_HIGH_SPEED, CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, 3, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)

// Enable EP IN flow control so TinyUSB paces mic packets to ~5.5125 samples/µframe.
// Without this, mic TX sends bursty packets (0,0,48,0,0,...) which breaks implicit
// feedback — the ASIO driver derives the device clock from per-µframe packet sizes.
#define CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL 1
#endif

// Max EP size across all alt settings (24-bit is largest)
#define CFG_TUD_AUDIO_FUNC_1_EP_SZ_OUT CFG_TUD_AUDIO_FUNC_1_EP_SZ_OUT_24

// Software buffers - 32x max EP size for High-Speed (read FIFO every 1ms = 8 HS frames)
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ (32 * CFG_TUD_AUDIO_FUNC_1_EP_SZ_OUT)
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX CFG_TUD_AUDIO_FUNC_1_EP_SZ_OUT

#if CFG_TUD_AUDIO_ENABLE_EP_IN
#define CFG_TUD_AUDIO_FUNC_1_EP_SZ_IN  CFG_TUD_AUDIO_FUNC_1_EP_SZ_IN_24

#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ (32 * CFG_TUD_AUDIO_FUNC_1_EP_SZ_IN)
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX CFG_TUD_AUDIO_FUNC_1_EP_SZ_IN
#endif

#ifdef __cplusplus
}
#endif
