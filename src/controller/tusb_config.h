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
//
// *** PERFORMANCE WARNING ***
// CFG_TUSB_DEBUG >= 2 activates TU_LOG2() calls inside the BRDY interrupt
// handler (process_pipe_brdy, pipe_xfer_out/in).  At bInterval=1 the BRDY
// ISR fires ~16,000 times/sec across both ISO audio pipes.  Each TU_LOG2
// call performs a vsnprintf into a 256-byte stack buffer plus an RTT_Write —
// roughly 2–5 µs/call × 3 calls × 16,000/sec ≈ 10–30% of the CPU budget
// consumed purely by debug logging.  This starves the main loop, delays
// pad_led_flush_dirty(), and is the confirmed root cause of visual pad
// glitches at bInterval=1.  bInterval=4 was a workaround that reduced BRDY
// rate 4×, lowering logging overhead to ~8%, but did not fix the root cause.
//
// Keep at 0 for production builds.  Set to 3 only for active USB debugging.
#ifndef CFG_TUSB_DEBUG
#  ifdef ENABLE_RTT
#    define CFG_TUSB_DEBUG 3
#  else
#    define CFG_TUSB_DEBUG 0
#  endif
#endif

// Route TinyUSB debug output to Segger RTT channel 0.
// Only compiled when debug output is actually enabled — the extern reference
// to SEGGER_RTT_Write would be an unresolved symbol if RTT is not linked.
#if CFG_TUSB_DEBUG > 0
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
#endif // CFG_TUSB_DEBUG > 0

// Enable Device stack
#define CFG_TUD_ENABLED 1

// RHPort configuration
#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE
#define CFG_TUSB_RHPORT1_MODE 0

// RHPort number used for device
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT 0
#endif

// RHPort max operational speed
#ifndef BOARD_TUD_MAX_SPEED
#define BOARD_TUD_MAX_SPEED OPT_MODE_FULL_SPEED
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
// RX must be large enough that _prep_out_transaction (which requires
// remaining >= EP_BUFSIZE=512) can always submit the next OUT transfer
// even when previously received data hasn't been consumed yet.
// The host-demo sends ~1200 bytes/frame, so 2048 provides safe headroom.
#define CFG_TUD_CDC_RX_BUFSIZE 2048
#define CFG_TUD_CDC_TX_BUFSIZE 512

// CDC Endpoint transfer buffer size: 512 for HS bulk, 64 for FS bulk
#if BOARD_TUD_MAX_SPEED == OPT_MODE_HIGH_SPEED
#define CFG_TUD_CDC_EP_BUFSIZE 512
#else
#define CFG_TUD_CDC_EP_BUFSIZE 64
#endif

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

// bInterval for HS audio ISO data endpoints.
//
// bInterval=1 (every 125 µs) fires 8,000 BRDY/sec per ISO pipe (16,000 total
// across speaker OUT + mic/feedback IN).  With CFG_TUSB_DEBUG=0 the actual
// USB work per BRDY is ~3–5 µs (hardware FIFO read at MBW-32), totalling
// ~5–8% CPU at bInterval=1 — comfortably below budget.
//
// bInterval=4 was a workaround for the CFG_TUSB_DEBUG=3 logging overhead
// (~32% CPU at bInterval=1 with debug=3, ~8% at bInterval=4).  With debug
// fixed at 0, bInterval=1 is both correct and achievable.
//
// Must match the _interval field in usb_descriptors.h (now driven from here).
#define CFG_TUD_AUDIO_HS_BINTERVAL 1u

// EP max packet size accounting for bInterval.
// For HS ISO: the packet period is bInterval microframes of 125 µs each.
// ceil(freq × bInterval / 8000) gives the maximum samples per packet;
// +1 sample of safety margin covers the async clock jitter per USB audio spec.
#define _TUSB_AUDIO_HS_EP_SZ(_freq, _bytes, _ch)                                                                       \
	((((((_freq) * CFG_TUD_AUDIO_HS_BINTERVAL) + 7999u) / 8000u) + 1u) * (_bytes) * (_ch))

// At bInterval=1: ceil(44100/8000)+1 = 7 stereo frames.
//   16-bit: 7 × 2 × 2 =  28 bytes/packet
//   24-bit: 7 × 3 × 2 =  42 bytes/packet
// At bInterval=4 (previous): 24× larger packets (144 bytes 24-bit).
// FS: bInterval fixed at 1ms, use standard TUD_AUDIO_EP_SIZE formula.
#if TUD_OPT_HIGH_SPEED
#  define CFG_TUD_AUDIO_FUNC_1_EP_SZ_OUT_16 \
	_TUSB_AUDIO_HS_EP_SZ(CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, 2, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)
#  define CFG_TUD_AUDIO_FUNC_1_EP_SZ_OUT_24 \
	_TUSB_AUDIO_HS_EP_SZ(CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, 3, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)
#else
#  define CFG_TUD_AUDIO_FUNC_1_EP_SZ_OUT_16 \
	TUD_AUDIO_EP_SIZE(0, CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, 2, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)
#  define CFG_TUD_AUDIO_FUNC_1_EP_SZ_OUT_24 \
	TUD_AUDIO_EP_SIZE(0, CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, 3, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_IN
#if TUD_OPT_HIGH_SPEED
#  define CFG_TUD_AUDIO_FUNC_1_EP_SZ_IN_16 \
	_TUSB_AUDIO_HS_EP_SZ(CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, 2, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)
#  define CFG_TUD_AUDIO_FUNC_1_EP_SZ_IN_24 \
	_TUSB_AUDIO_HS_EP_SZ(CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, 3, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)
#else
#  define CFG_TUD_AUDIO_FUNC_1_EP_SZ_IN_16 \
	TUD_AUDIO_EP_SIZE(0, CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, 2, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)
#  define CFG_TUD_AUDIO_FUNC_1_EP_SZ_IN_24 \
	TUD_AUDIO_EP_SIZE(0, CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, 3, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)
#endif

// Enable EP IN flow control so TinyUSB paces mic packets to ~5.5125 samples/µframe.
// Without this, mic TX sends bursty packets (0,0,48,0,0,...) which breaks implicit
// feedback — the ASIO driver derives the device clock from per-µframe packet sizes.
#define CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL 1
#endif

// Max EP size across all alt settings (24-bit is largest)
#define CFG_TUD_AUDIO_FUNC_1_EP_SZ_OUT CFG_TUD_AUDIO_FUNC_1_EP_SZ_OUT_24

// Software FIFO buffers — 32× EP size provides ~4 ms of audio headroom.
// At bInterval=1 (42 bytes/packet 24-bit): 32×42 = 1344 bytes (~32 packets).
// The main loop drains the FIFO into the SSI DMA ring each iteration (≤1 ms
// typical), so 32 packets of slack is ample.
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
