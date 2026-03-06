#include "controller_util.h"
#include "display_control.h"
#include "drivers/pic/pic.h"

extern "C" {
#include "RTT/SEGGER_RTT.h"
#include "RZA1/gpio/gpio.h"
#include "RZA1/mtu/mtu.h"
#include "drivers/oled/oled_low_level.h"
#include "RZA1/uart/sio_char.h"
#include "definitions.h"
#include "drivers/oled/oled.h"
#include "drivers/uart/uart.h"
#include "util/cfunctions.h"
#include <string.h>
}

// External OLED framebuffer (defined in oled_stubs.c)
extern "C" uint8_t oledMainImage[OLED_MAIN_HEIGHT_PIXELS >> 3][OLED_MAIN_WIDTH_PIXELS];

// Pad LED state tracking (18 columns x 8 rows, RGB)
// 16 main grid columns + 2 sidebar columns = 9 column pairs
static RGB pad_led_state[18][8];

// Dirty bitmask for column pairs — bit N = column pair N needs PIC update
// 9 pairs total (pairs 0-8), so uint16_t is needed
static uint16_t pad_col_pair_dirty = 0;

// Gate output GPIO pins (from cv_engine.h)
// Port 2: pins 7, 8, 9 (gates 0, 1, 2)
// Port 4: pin 0 (gate 3)
static constexpr uint8_t gatePort[] = {2, 2, 2, 4};
static constexpr uint8_t gatePin[] = {7, 8, 9, 0};

extern "C" {

// OLED framebuffer: 128x64 pixels = 1024 bytes (8 rows of 128 bytes)
uint8_t oledMainImage[OLED_MAIN_HEIGHT_PIXELS >> 3][OLED_MAIN_WIDTH_PIXELS] = {{0}};

void cv_gate_init(void) {
	// CV DAC (MAX5136) linearity initialization sequence — required by datasheet.
	// Mirrors CVEngine::init() in src/deluge/processing/engines/cv_engine.cpp.
	// The controller always has an OLED so we always route through enqueueCVMessage.
	enqueueCVMessage(SPI_CHANNEL_CV, 0b00000101000000100000000000000000); // LIN = 1
	delay_ms(10);
	enqueueCVMessage(SPI_CHANNEL_CV, 0b00000101000000000000000000000000); // LIN = 0

	// Configure gate output GPIO pins and set all gates to OFF (V-trig: high = off).
	// Matches CVEngine::init() gate setup loop.
	for (int i = 0; i < 4; i++) {
		setPinAsOutput(gatePort[i], gatePin[i]);
		setOutputState(gatePort[i], gatePin[i], 1); // 1 = gate OFF for V-trig
	}
}

void display_update(const uint8_t* buffer, uint16_t size) {
	// Validate size
	if (size > sizeof(oledMainImage)) {
		size = sizeof(oledMainImage);
	}

	// Copy framebuffer data — always write latest content first so any pending
	// transfer picks it up, since oledMainImage is always the source.
	memcpy(oledMainImage, buffer, size);

	// Coalesce OLED transfers: only enqueue if there is no OLED entry already
	// sitting in the SPI queue. Without this guard, rapid calls (e.g. 30fps) fill
	// the 32-slot circular queue until writePos laps readPos — stalling the pipeline.
	// Since the source is always oledMainImage (updated above), an already-queued
	// transfer will send the latest content, so no frame is lost.
	if (!spiHasPendingOledTransfer()) {
		enqueueSPITransfer(0, (uint8_t const*)oledMainImage);
	}
}

void display_clear() {
	// Clear framebuffer
	memset(oledMainImage, 0, sizeof(oledMainImage));

	// Queue the blank image for transfer to OLED
	// The oledRoutine() will handle select/deselect automatically
	enqueueSPITransfer(0, (uint8_t const*)oledMainImage);
}
void pad_led_set_rgb(uint8_t col, uint8_t row, uint8_t r, uint8_t g, uint8_t b) {
	// Deluge has 18 columns x 8 rows (16 main grid + 2 sidebar)
	if (col >= 18 || row >= 8) {
		return;
	}

	// Update state and mark column pair dirty — actual PIC send deferred to pad_led_flush_dirty()
	pad_led_state[col][row].r = r;
	pad_led_state[col][row].g = g;
	pad_led_state[col][row].b = b;
	pad_col_pair_dirty |= (1u << (col / 2));
}

void pad_led_flush_dirty(void) {
	if (pad_col_pair_dirty == 0) {
		return;
	}

	// UART throughput gate: the PIC UART runs at 200kbaud (10 bits/byte = 20,000 bytes/sec).
	// A full 9-pair update is 441 bytes, taking 22.05ms to transmit.  The host sends at 60fps
	// which would require 26,460 bytes/sec — 32% more than the UART can drain.  Without rate
	// limiting the TX ring buffer fills to capacity over ~150ms and no pairs can ever be sent.
	//
	// Gate sends to at most ~43fps (23ms minimum interval ≈ 12,000 fast-timer ticks at 520kHz).
	// 441 bytes × 43fps = 18,963 bytes/sec < 20,000 bytes/sec — sustainable.
	// The dirty-state accumulates between sends so the pads always show the latest colour.
	static constexpr uint16_t kFlushGateTicks = 12000u; // ~23ms at 520kHz
	static uint16_t last_flush_tick = 0;
	uint16_t now = *TCNT[TIMER_SYSTEM_FAST];
	if ((uint16_t)(now - last_flush_tick) < kFlushGateTicks) {
		return;
	}
	last_flush_tick = now;

	// Each column-pair costs 1 command byte + 16 * 3 RGB bytes = 49 bytes in the PIC UART TX ring
	// buffer. Guard against overflowing the 1024-byte ring (which would silently corrupt queued
	// commands) by stopping early if there is not enough space.
	static constexpr int32_t kBytesPerPair = 1 + 16 * 3; // 49
	static constexpr int32_t kPairGuard    = kBytesPerPair + 8;

	// Rotate the starting pair each call so that no single pair is perpetually last.
	static uint8_t start_pair = 0;

#if ENABLE_RTT
	// ── Diagnostics ─────────────────────────────────────────────────
	static constexpr uint32_t kDiagInterval = 45; // ~1 s at 43fps gate
	static uint32_t diag_flush_calls   = 0;
	static uint32_t diag_send[9]       = {};
	static uint32_t diag_skip[9]       = {};
	static uint32_t diag_leftover      = 0;
	static uint32_t diag_min_space     = 1024;
	// ────────────────────────────────────────────────────────────────

	++diag_flush_calls;
#endif

	for (uint8_t i = 0; i < 9; i++) {
		uint8_t pair = (start_pair + i) % 9;

		if (!(pad_col_pair_dirty & (1u << pair))) {
			continue;
		}

		int32_t space = uartGetTxBufferSpace(UART_ITEM_PIC);
#if ENABLE_RTT
		if ((uint32_t)space < (uint32_t)diag_min_space) {
			diag_min_space = (uint32_t)space;
		}

		if (space < kPairGuard) {
			diag_skip[pair]++;
			break;
		}
#endif

		uint8_t base_col = pair * 2;
		std::array<RGB, 16> colours;
		for (uint8_t row_idx = 0; row_idx < 8; row_idx++) {
			colours[row_idx] = pad_led_state[base_col][row_idx];
		}
		for (uint8_t row_idx = 0; row_idx < 8; row_idx++) {
			colours[8 + row_idx] = pad_led_state[base_col + 1][row_idx];
		}
		PIC::setColourForTwoColumns(pair, colours);
		pad_col_pair_dirty &= ~(1u << pair);
#if ENABLE_RTT
		diag_send[pair]++;
#endif
	}

#if ENABLE_RTT
	if (pad_col_pair_dirty) {
		diag_leftover++;
	}
#endif

	start_pair = (start_pair + 1) % 9;
	PIC::flush();

	#if ENABLE_RTT
	// ── Periodic diagnostic dump ─────────────────────────────────────
	if (diag_flush_calls >= kDiagInterval) {
		const RGB& tr = pad_led_state[17][7];
		CDBG_FMT(
			"[PAD DIAG] flush_calls=%lu leftover=%lu min_uart_space=%lu\r\n"
			"  pair sends:  %2lu %2lu %2lu %2lu %2lu %2lu %2lu %2lu %2lu\r\n"
			"  pair skips:  %2lu %2lu %2lu %2lu %2lu %2lu %2lu %2lu %2lu\r\n"
			"  top-right (col17,row7): R=%u G=%u B=%u\r\n",
			(unsigned long)diag_flush_calls,
			(unsigned long)diag_leftover,
			(unsigned long)diag_min_space,
			(unsigned long)diag_send[0], (unsigned long)diag_send[1],
			(unsigned long)diag_send[2], (unsigned long)diag_send[3],
			(unsigned long)diag_send[4], (unsigned long)diag_send[5],
			(unsigned long)diag_send[6], (unsigned long)diag_send[7],
			(unsigned long)diag_send[8],
			(unsigned long)diag_skip[0], (unsigned long)diag_skip[1],
			(unsigned long)diag_skip[2], (unsigned long)diag_skip[3],
			(unsigned long)diag_skip[4], (unsigned long)diag_skip[5],
			(unsigned long)diag_skip[6], (unsigned long)diag_skip[7],
			(unsigned long)diag_skip[8],
			(unsigned)tr.r, (unsigned)tr.g, (unsigned)tr.b);

		diag_flush_calls  = 0;
		diag_leftover     = 0;
		diag_min_space    = 1024;
		for (uint8_t p = 0; p < 9; p++) { diag_send[p] = 0; diag_skip[p] = 0; }
	}
#endif
	// ────────────────────────────────────────────────────────────────
}

void pad_led_set_all(const uint8_t* data, uint16_t len) {
	// Bulk set all pads: data layout is col-major, 18 cols x 8 rows x 3 bytes (R,G,B) = 432 bytes
	const uint16_t expected = 18 * 8 * 3; // 432
	if (len < expected) {
		return;
	}

	const uint8_t* p = data;
	for (auto & col : pad_led_state) {
		for (auto & row : col) {
			row.r = p[0];
			row.g = p[1];
			row.b = p[2];
			p += 3;
		}
	}

	// Mark all 9 column pairs dirty; the main loop will flush them
	// as UART buffer space opens up (see pad_led_flush_dirty backpressure).
	pad_col_pair_dirty = 0x1FF;
}

void pad_led_clear_all() {
	// Clear all 18 columns (16 main grid + 2 sidebar) by sending 9 column pairs
	std::array<RGB, 16> zero_colors{}; // All black
	for (uint8_t pair = 0; pair < 9; ++pair) {
		PIC::setColourForTwoColumns(pair, zero_colors);
	}
	PIC::flush();
	PIC::waitForFlush();
}

void hardware_led_set(uint8_t led_index, bool on) {
	// Map LED index (0-35) to PIC command (152-223)
	// OFF: 152-187, ON: 188-223
	if (led_index >= 36) {
		return;
	}

	if (on) {
		PIC::setLEDOn(led_index);
	}
	else {
		PIC::setLEDOff(led_index);
	}
	PIC::flush();
}
void cv_set(uint8_t channel, uint16_t value) {
	// Deluge has 4 CV output channels
	if (channel >= 4) {
		return;
	}

	// MAX5136 DAC protocol (matches CVEngine::sendVoltageOut in cv_engine.cpp):
	//   Bits 31-24: 0b00110000 | (1 << channel)  — write command + channel select bitmask
	//   Bits 23-8:  16-bit voltage value (0–65535, where 6552 ≈ 1V)
	//   Bits  7-0:  zero padding
	uint32_t message = (uint32_t)(0b00110000 | (1 << channel)) << 24;
	message |= (uint32_t)value << 8;
	enqueueCVMessage(channel, message);
}

void gate_set(uint8_t channel, bool on) {
	// Deluge has 4 Gate output channels
	if (channel >= 4) {
		return;
	}

	// Gate outputs are direct GPIO pins
	// Note: setOutputState is inverted - sending true (1) turns the gate OFF
	// So we need to invert the 'on' value
	setOutputState(gatePort[channel], gatePin[channel], on ? 0 : 1);
}

} // extern "C"
