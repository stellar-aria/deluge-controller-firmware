

#include "usb_audio.h"
#include "controller_util.h"
#include "definitions_cxx.hpp"  // For Pin constants (SPEAKER_ENABLE, HEADPHONE_DETECT, etc.)
#include "definitions.h"
#include "drivers/ssi/ssi.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "RZA1/mtu/mtu.h"
#include <string.h>

extern "C" {
#include "RZA1/gpio/gpio.h"
}

#if CFG_TUD_AUDIO

// Audio streaming state - speaker (output)
static volatile bool audio_spk_streaming_active = false;
static volatile uint8_t audio_spk_alt_setting = 0;
static volatile uint8_t audio_spk_bytes_per_sample = 2; // 2 for 16-bit (alt=1), 3 for 24-bit (alt=2)

#if CFG_TUD_AUDIO_ENABLE_EP_IN
// Audio streaming state - microphone (input) — only in headset mode (mode 0)
static volatile bool audio_mic_streaming_active = false;
static volatile uint8_t audio_mic_alt_setting = 0;
static volatile uint8_t audio_mic_bytes_per_sample = 2; // 2 for 16-bit (alt=1), 3 for 24-bit (alt=2)
#endif

// Write position tracking for the circular SSI TX buffer (speaker)
static int32_t* audio_write_pos = NULL;

#if CFG_TUD_AUDIO_ENABLE_EP_IN
// Read position tracking for the circular SSI RX buffer (microphone)
static int32_t* audio_read_pos = NULL;
#endif

// Flag to defer TX buffer dither fill from ISR to main loop
static volatile bool audio_needs_dither_fill = false;

// Hardware-timer snapshot of last audio RX (TIMER_SYSTEM_SLOW, 33kHz)
static volatile uint16_t audio_spk_last_rx_hw_time = 0;

// Stream timeout in hardware-timer ticks: 200ms × 33 ticks/ms = 6600
#define AUDIO_STREAM_TIMEOUT_HW_TICKS 6600

// How far ahead of DMA position to write (in stereo sample pairs).
// Must be large enough to survive the worst-case main-loop latency.
// With a 1024-sample TX buffer, 256 pairs ≈ 5.8 ms gives plenty of margin.
#define AUDIO_WRITE_AHEAD_SAMPLES 256

#if CFG_TUD_AUDIO_ENABLE_EP_IN
// How far behind DMA position to read (in stereo sample pairs)
#define AUDIO_READ_BEHIND_SAMPLES 32

// Number of stereo samples to process per main-loop iteration for mic TX
// At 44.1kHz, we need 44 stereo sample-pairs per ms; process a chunk each loop pass
#define MIC_SAMPLES_PER_CHUNK 48
#endif

// Simple LFSR-based noise generator to prevent codec DC auto-mute
// The codec chip auto-mutes when it sees continuous all-zero samples
static uint32_t dither_lfsr = 0xACE1u;

static inline int32_t get_dither_noise(void) {
	// 16-bit Galois LFSR - produces low-level pseudo-random noise
	uint32_t bit = dither_lfsr & 1u;
	dither_lfsr >>= 1;
	if (bit) {
		dither_lfsr ^= 0xB400u; // Taps at positions 16,14,13,11
	}
	// Return tiny noise value in the lowest bits of the 24-bit range
	// This is inaudible but prevents the codec from entering auto-mute
	return (int32_t)(dither_lfsr & 0x1F) - 0x10; // Range: -16 to +15
}

// Helper to advance write position with wrap-around
static inline int32_t* advance_write_pos(int32_t* pos, int32_t count, int32_t* bufStart, int32_t* bufEnd) {
	pos += count;
	int32_t bufSize = bufEnd - bufStart;
	while (pos >= bufEnd) {
		pos -= bufSize;
	}
	return pos;
}

// Helper to advance read position with wrap-around
static inline int32_t* advance_read_pos(int32_t* pos, int32_t count, int32_t* bufStart, int32_t* bufEnd) {
	pos += count;
	int32_t bufSize = bufEnd - bufStart;
	while (pos >= bufEnd) {
		pos -= bufSize;
	}
	while (pos < bufStart) {
		pos += bufSize;
	}
	return pos;
}

void usb_audio_init(void) {
	audio_spk_streaming_active = false;
	audio_spk_alt_setting = 0;
#if CFG_TUD_AUDIO_ENABLE_EP_IN
	audio_mic_streaming_active = false;
	audio_mic_alt_setting = 0;
#endif

	// Initialize write position to start of TX buffer
	audio_write_pos = getTxBufferStart();

#if CFG_TUD_AUDIO_ENABLE_EP_IN
	// Initialize read position to start of RX buffer
	audio_read_pos = getRxBufferStart();
#endif

	// Fill the SSI TX buffer with dither noise (prevents codec DC auto-mute)
	int32_t* txBuffer = getTxBufferStart();
	int32_t* txBufferEnd = getTxBufferEnd();
	for (int32_t* p = txBuffer; p < txBufferEnd; p++) {
		*p = get_dither_noise();
	}

	TU_LOG2("USB Audio initialized (speaker%s)\r\n",
	        CFG_TUD_AUDIO_ENABLE_EP_IN ? " + microphone" : " + feedback EP");
}

void usb_audio_task(void) {
	// Deferred dither fill — moved out of ISR-context set_itf callback
	if (audio_needs_dither_fill) {
		audio_needs_dither_fill = false;
		int32_t* txBuffer = getTxBufferStart();
		int32_t* txBufferEnd = getTxBufferEnd();
		for (int32_t* p = txBuffer; p < txBufferEnd; p++) {
			*p = get_dither_noise();
		}
	}

	// Check if speaker stream has timed out (hardware timer based)
	if (audio_spk_streaming_active) {
		uint16_t now = *TCNT[TIMER_SYSTEM_SLOW];
		uint16_t elapsed = now - audio_spk_last_rx_hw_time;
		if (elapsed > AUDIO_STREAM_TIMEOUT_HW_TICKS) {
			CDBG_STR("[SPK] stream timed out\n");
			audio_spk_streaming_active = false;
			usb_audio_update_speaker_state();

			// Fill TX buffer with dither to stop the old audio looping as a buzz
			audio_needs_dither_fill = true;
		}
	}

	// ---- Speaker RX: read from USB and write to SSI TX buffer ----
	if (audio_spk_alt_setting != 0) {
		// Read all available data from TinyUSB FIFO in a loop
		// At HS each microframe delivers ~28 bytes; we drain whatever is buffered
		static uint8_t spk_buf[252]; // 252 = 42 stereo frames × 6 bytes — always frame-aligned for 24-bit
		static uint32_t spk_dbg_counter = 0;
		// Round read size to a multiple of the frame size so we never discard
		// partial samples and misalign subsequent reads.
		uint16_t frame_bytes = audio_spk_bytes_per_sample * NUM_MONO_OUTPUT_CHANNELS; // 4 or 6
		uint16_t max_read = (sizeof(spk_buf) / frame_bytes) * frame_bytes;
		uint16_t bytes_read;
		while ((bytes_read = tud_audio_n_read(0, spk_buf, max_read)) > 0) {
			if ((spk_dbg_counter++ % 50000) == 0) {
				int32_t* dmaPos = (int32_t*)getTxBufferCurrentPlace();
				int32_t* txBuf = getTxBufferStart();
				int32_t dmaOff = dmaPos - txBuf;
				int32_t wrOff = audio_write_pos - txBuf;
				CDBG_FMT("[SPK] read %u bytes  dma=%d wr=%d\n",
					bytes_read, (int)dmaOff, (int)wrOff);
			}
			// Update streaming state — use hardware timer for timeout
			audio_spk_last_rx_hw_time = *TCNT[TIMER_SYSTEM_SLOW];
			bool was_streaming = audio_spk_streaming_active;
			audio_spk_streaming_active = true;

			int32_t* txBuffer = getTxBufferStart();
			int32_t* txBufferEnd = getTxBufferEnd();

			if (!was_streaming) {
				// Reset write position ahead of current DMA position
				int32_t* dmaPos = (int32_t*)getTxBufferCurrentPlace();
				audio_write_pos = advance_write_pos(dmaPos,
				    AUDIO_WRITE_AHEAD_SAMPLES * NUM_MONO_OUTPUT_CHANNELS, txBuffer, txBufferEnd);

				CDBG_STR("[SPK] streaming started\n");
				usb_audio_update_speaker_state();
			}

			// Convert USB samples to MSB-aligned 32-bit for CODEC.
			// The CODEC expects audio in upper bits of each 32-bit SSI word.
			int32_t* writePos = audio_write_pos;

			if (audio_spk_bytes_per_sample == 2) {
				// 16-bit: sample << 16 puts it in bits 16-31 (MSB-aligned)
				int16_t* samples_16 = (int16_t*)spk_buf;
				int32_t numSamples = bytes_read / sizeof(int16_t);

				for (int32_t i = 0; i < numSamples; i++) {
					*writePos = ((int32_t)samples_16[i]) << 16;
					writePos++;
					if (writePos >= txBufferEnd) {
						writePos = txBuffer;
					}
				}
			}
			else {
				// 24-bit: 3 bytes per sample, little-endian from USB.
				// Reassemble and shift << 8 to MSB-align in 32-bit word.
				uint8_t* src = spk_buf;
				int32_t numSamples = bytes_read / 3;

				for (int32_t i = 0; i < numSamples; i++) {
					int32_t sample = (int32_t)((uint32_t)src[0] | ((uint32_t)src[1] << 8) | ((uint32_t)src[2] << 16));
					// Sign-extend from 24-bit
					if (sample & 0x800000) {
						sample |= (int32_t)0xFF000000;
					}
					int32_t val = sample << 8; // MSB-align: bits 8-31
					*writePos = val;

					src += 3;
					writePos++;
					if (writePos >= txBufferEnd) {
						writePos = txBuffer;
					}
				}
			}

			audio_write_pos = writePos;
		}

		// ---- Underrun detection ----------------------------------------
		// If the main loop was slow, the DMA may have caught up to (or
		// passed) the write position.  Detect this and re-sync the write
		// pointer ahead of the DMA so that subsequent samples land safely.
		{
			int32_t* txBuffer  = getTxBufferStart();
			int32_t* txBufEnd  = getTxBufferEnd();
			int32_t  bufSize   = txBufEnd - txBuffer;
			int32_t  dmaOff    = (int32_t*)getTxBufferCurrentPlace() - txBuffer;
			int32_t  wrOff     = audio_write_pos - txBuffer;
			int32_t  ahead     = (wrOff - dmaOff + bufSize) % bufSize;

			// 'ahead' is how many mono-sample slots the write pointer is in
			// front of the DMA.  If it drops below half the write-ahead
			// threshold we have (or are about to have) an underrun.
			const int32_t minAhead = AUDIO_WRITE_AHEAD_SAMPLES * NUM_MONO_OUTPUT_CHANNELS / 2;
			if (ahead < minAhead) {
				int32_t* dmaPos = (int32_t*)getTxBufferCurrentPlace();
				audio_write_pos = advance_write_pos(dmaPos,
				    AUDIO_WRITE_AHEAD_SAMPLES * NUM_MONO_OUTPUT_CHANNELS,
				    txBuffer, txBufEnd);
				// Rate-limit underrun logging to avoid stalling the main loop.
				// (RTT is non-blocking now, but still avoid flooding the channel.)
				static uint32_t underrun_count = 0;
				underrun_count++;
				if ((underrun_count & 0xFF) == 1) {
					CDBG_FMT("[SPK] underrun #%u  ahead=%d\n",
						(unsigned)underrun_count, (int)ahead);
				}
			}
		}

		// ---- Explicit feedback EP: rate control via SSI TX buffer level --
		// The feedback value tells the host how many samples per microframe
		// the device is consuming.  We derive it from the distance between
		// the write pointer and the DMA read pointer in the SSI TX buffer.
		// This is a direct measure of the host-vs-codec rate mismatch.
#if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
		if (audio_spk_streaming_active) {
			int32_t* txBuffer  = getTxBufferStart();
			int32_t* txBufEnd  = getTxBufferEnd();
			int32_t  bufSize   = txBufEnd - txBuffer;
			int32_t  dmaOff    = (int32_t*)getTxBufferCurrentPlace() - txBuffer;
			int32_t  wrOff     = audio_write_pos - txBuffer;
			int32_t  ahead     = (wrOff - dmaOff + bufSize) % bufSize;

			// Target: half the buffer (in mono slots).  This gives equal
			// headroom for both underrun and overrun.
			int32_t target = bufSize / 2;
			int32_t error  = ahead - target; // +ve = too far ahead (host too fast)

			// Nominal feedback: 44100 / 8000 = 5.5125 in 16.16 format.
			// 44100 * 65536 / 8000 = 361267
			static const uint32_t fb_nominal = (uint32_t)((44100ULL << 16) / 8000);

			// Proportional gain.  Each unit of 'error' is one mono sample
			// slot = 1/44100 s.  Divide by 64 for smooth convergence:
			//   1 slot error  → ~0.00024 extra samples / µframe
			//   64 slot error → ~1 extra sample / µframe
			int32_t fb_delta = error / 64;

			uint32_t fb = (uint32_t)((int32_t)fb_nominal - fb_delta);
			// Clamp to ±1 sample from nominal (USB spec requirement)
			if (fb > (6u << 16)) fb = 6u << 16;
			if (fb < (5u << 16)) fb = 5u << 16;

			tud_audio_n_fb_set(0, fb);
		}
#endif // CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
	}

	// ---- Microphone TX: read from SSI RX buffer and write to USB ----
#if CFG_TUD_AUDIO_ENABLE_EP_IN
	if (audio_mic_streaming_active && audio_mic_alt_setting != 0) {
		int32_t* rxBuffer = getRxBufferStart();
		int32_t* rxBufferEnd = getRxBufferEnd();
		int32_t bufSize = rxBufferEnd - rxBuffer;

		// Get current DMA position (where ADC data is being written by hardware)
		int32_t* dmaPos = (int32_t*)getRxBufferCurrentPlace();

		// Calculate the safe read limit (behind DMA position)
		int32_t* safeReadPos = advance_read_pos(dmaPos,
		    -AUDIO_READ_BEHIND_SAMPLES * NUM_MONO_INPUT_CHANNELS, rxBuffer, rxBufferEnd);

		// Calculate how many samples are available
		int32_t readOffset = audio_read_pos - rxBuffer;
		int32_t safeOffset = safeReadPos - rxBuffer;
		int32_t available = (safeOffset - readOffset + bufSize) % bufSize;

		// Limit to a reasonable chunk per loop pass
		int32_t maxSamples = MIC_SAMPLES_PER_CHUNK * NUM_MONO_INPUT_CHANNELS;
		int32_t samplesToSend = (available < maxSamples) ? available : maxSamples;

		if (samplesToSend > 0) {
			int32_t* readPos = audio_read_pos;

			if (audio_mic_bytes_per_sample == 2) {
				// 16-bit: convert 24-bit SSI samples to 16-bit USB samples
				static int16_t usb_tx_buffer_16[MIC_SAMPLES_PER_CHUNK * NUM_MONO_INPUT_CHANNELS];

				for (int32_t i = 0; i < samplesToSend; i++) {
					usb_tx_buffer_16[i] = (int16_t)(*readPos >> 8);
					readPos++;
					if (readPos >= rxBufferEnd) {
						readPos = rxBuffer;
					}
				}

				audio_read_pos = readPos;
				tud_audio_n_write(0, usb_tx_buffer_16, samplesToSend * sizeof(int16_t));
			}
			else {
				// 24-bit: extract 24-bit from 32-bit SSI word, pack as 3 bytes little-endian
				static uint8_t usb_tx_buffer_24[MIC_SAMPLES_PER_CHUNK * NUM_MONO_INPUT_CHANNELS * 3];
				uint8_t* dst = usb_tx_buffer_24;

				for (int32_t i = 0; i < samplesToSend; i++) {
					int32_t sample = *readPos >> 8; // Shift down from MSB-aligned to 24-bit
					dst[0] = (uint8_t)(sample & 0xFF);
					dst[1] = (uint8_t)((sample >> 8) & 0xFF);
					dst[2] = (uint8_t)((sample >> 16) & 0xFF);
					dst += 3;
					readPos++;
					if (readPos >= rxBufferEnd) {
						readPos = rxBuffer;
					}
				}

				audio_read_pos = readPos;
				tud_audio_n_write(0, usb_tx_buffer_24, samplesToSend * 3);
			}
		}
	}
#endif // CFG_TUD_AUDIO_ENABLE_EP_IN
}

bool usb_audio_is_streaming(void) {
	return audio_spk_streaming_active
#if CFG_TUD_AUDIO_ENABLE_EP_IN
	       || audio_mic_streaming_active
#endif
	    ;
}

void usb_audio_update_speaker_state(void) {
	// Check if headphones are plugged in
	bool headphonesPluggedIn = readInput(HEADPHONE_DETECT.port, HEADPHONE_DETECT.pin) != 0;

	// Check if line out is plugged in
	bool lineOutPluggedInL = readInput(LINE_OUT_DETECT_L.port, LINE_OUT_DETECT_L.pin) != 0;
	bool lineOutPluggedInR = readInput(LINE_OUT_DETECT_R.port, LINE_OUT_DETECT_R.pin) != 0;

	// Enable speaker only if streaming AND no headphones/line out
	bool enableSpeaker =
	    audio_spk_streaming_active && !headphonesPluggedIn && !lineOutPluggedInL && !lineOutPluggedInR;

	setOutputState(SPEAKER_ENABLE.port, SPEAKER_ENABLE.pin, enableSpeaker ? 1 : 0);

	CDBG_FMT("[SPK] speaker=%s stream=%d hp=%d lineL=%d lineR=%d\n",
		enableSpeaker ? "ON" : "OFF",
		audio_spk_streaming_active, headphonesPluggedIn, lineOutPluggedInL, lineOutPluggedInR);

	TU_LOG2("Speaker: %s (stream=%d, hp=%d, lineL=%d, lineR=%d)\r\n", enableSpeaker ? "ON" : "OFF",
	        audio_spk_streaming_active, headphonesPluggedIn, lineOutPluggedInL, lineOutPluggedInR);
}

// TinyUSB calls these callbacks from C linkage — must be extern "C" so the linker finds them
// by their unmangled names.
extern "C" {

// ISR notification that audio data arrived — we just note it, actual read happens in usb_audio_task
bool tud_audio_rx_done_isr(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out,
                           uint8_t cur_alt_setting) {
	(void)rhport;
	(void)func_id;
	(void)ep_out;
	(void)n_bytes_received;
	(void)cur_alt_setting;
	// Data will be read in usb_audio_task() from the main loop
	return true;
}

#if CFG_TUD_AUDIO_ENABLE_EP_IN
// TX ISR callback - called when TinyUSB has sent microphone data to the host.
// Keep this minimal — actual data filling happens in usb_audio_task() in main loop.
bool tud_audio_tx_done_isr(uint8_t rhport, uint16_t n_bytes_sent, uint8_t func_id, uint8_t ep_in,
                           uint8_t cur_alt_setting) {
	(void)rhport;
	(void)n_bytes_sent;
	(void)func_id;
	(void)ep_in;
	(void)cur_alt_setting;
	// Data is pre-filled by usb_audio_task() via tud_audio_n_write().
	// Nothing to do here.
	return true;
}
#endif // CFG_TUD_AUDIO_ENABLE_EP_IN

// TinyUSB audio callbacks
bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const* p_request) {
	(void)rhport;

	uint8_t const itf = tu_u16_low(p_request->wIndex);
	uint8_t const alt = tu_u16_low(p_request->wValue);

	TU_LOG2("Audio set_itf: itf=%u, alt=%u\r\n", itf, alt);

	if (itf == ITF_NUM_AUDIO_STREAMING_SPK) {
		audio_spk_alt_setting = alt;
		CDBG_FMT("[SPK] set_itf alt=%u\n", alt);

		if (alt == 0) {
			// Speaker streaming stopped
			CDBG_STR("[SPK] streaming stopped\n");
			audio_spk_streaming_active = false;
			usb_audio_update_speaker_state();

			// Defer dither fill to main loop — keep this callback fast for USB timing
			audio_needs_dither_fill = true;
		}
		else {
			audio_spk_bytes_per_sample = (alt == 2) ? 3 : 2;
			CDBG_FMT("[SPK] streaming enabled alt=%u (%u-bit)\n", alt, audio_spk_bytes_per_sample * 8);
#if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
			// Set initial feedback to nominal rate so host starts sending
			// at approximately the right rate before our control loop kicks in.
			tud_audio_n_fb_set(0, (uint32_t)((44100ULL << 16) / 8000));
#endif
		}
	}
#if CFG_TUD_AUDIO_ENABLE_EP_IN
	else if (itf == ITF_NUM_AUDIO_STREAMING_MIC) {
		audio_mic_alt_setting = alt;
		CDBG_FMT("[MIC] set_itf alt=%u\n", alt);

		if (alt == 0) {
			// Microphone streaming stopped
			CDBG_STR("[MIC] streaming stopped\n");
			audio_mic_streaming_active = false;
		}
		else {
			// Microphone streaming started
			audio_mic_bytes_per_sample = (alt == 2) ? 3 : 2;
			CDBG_FMT("[MIC] streaming enabled alt=%u (%u-bit)\n", alt, audio_mic_bytes_per_sample * 8);
			audio_mic_streaming_active = true;

			// Initialize read position behind current DMA position
			int32_t* rxBuffer = getRxBufferStart();
			int32_t* rxBufferEnd = getRxBufferEnd();
			int32_t* dmaPos = (int32_t*)getRxBufferCurrentPlace();
			audio_read_pos = advance_read_pos(dmaPos, -AUDIO_READ_BEHIND_SAMPLES * NUM_MONO_INPUT_CHANNELS, rxBuffer,
			                                  rxBufferEnd);
		}
	}
#endif // CFG_TUD_AUDIO_ENABLE_EP_IN

	return true;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request) {
	(void)rhport;
	(void)p_request;

	// Don't clear alt_setting or streaming state here.
	// TinyUSB calls close_ep BEFORE set_itf when switching between alt settings.
	// If we reset alt_setting=0 here, there's a window where usb_audio_task() skips
	// reading data, causing the host to retry and eventually give up.
	// All state management is handled by set_itf_cb and the stream timeout.
	return true;
}

#if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
// Feedback EP callback.
// We use DISABLED here and manually set the feedback value from
// usb_audio_task() based on the SSI TX buffer level (distance between
// the write pointer and the DMA read pointer).  This gives a direct
// measure of the actual host-vs-codec rate mismatch, unlike FIFO_COUNT
// which only sees the TinyUSB software FIFO level (always near zero
// because the main loop drains it instantly).
void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t* feedback_param) {
	(void)func_id;
	(void)alt_itf;
	feedback_param->method = AUDIO_FEEDBACK_METHOD_DISABLED;
	feedback_param->sample_freq = CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE;
}
#endif // CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP

// Audio control callbacks (volume, mute, etc.)
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* pBuff) {
	(void)rhport;
	(void)p_request;
	(void)pBuff;
	return true;
}

bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* pBuff) {
	(void)rhport;
	(void)p_request;
	(void)pBuff;
	return true;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const* p_request, uint8_t* pBuff) {
	(void)rhport;
	(void)pBuff;
	audio20_control_request_t const* request = (audio20_control_request_t const*)p_request;

	// Speaker feature unit set requests (volume/mute)
	if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT && request->bRequest == AUDIO20_CS_REQ_CUR) {
		if (request->bControlSelector == AUDIO20_FU_CTRL_MUTE) {
			return true;
		}
		else if (request->bControlSelector == AUDIO20_FU_CTRL_VOLUME) {
			return true;
		}
	}
#if CFG_TUD_AUDIO_ENABLE_EP_IN
	// Microphone feature unit set requests (volume/mute) - headset mode only
	else if (request->bEntityID == UAC2_ENTITY_MIC_FEATURE_UNIT && request->bRequest == AUDIO20_CS_REQ_CUR) {
		if (request->bControlSelector == AUDIO20_FU_CTRL_MUTE) {
			return true;
		}
		else if (request->bControlSelector == AUDIO20_FU_CTRL_VOLUME) {
			return true;
		}
	}
#endif
	// Clock source set requests
	else if (request->bEntityID == UAC2_ENTITY_CLOCK && request->bRequest == AUDIO20_CS_REQ_CUR) {
		if (request->bControlSelector == AUDIO20_CS_CTRL_SAM_FREQ) {
			uint32_t sr = 0;
			if (pBuff) sr = *(uint32_t*)pBuff;
			CDBG_FMT("[CLK] SET_CUR freq=%u\n", (unsigned)sr);
			return true;
		}
	}

	return false;
}

bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const* p_request) {
	(void)rhport;
	(void)p_request;
	return true;
}

bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const* p_request) {
	(void)rhport;
	(void)p_request;
	return true;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const* p_request) {
	audio20_control_request_t const* request = (audio20_control_request_t const*)p_request;

	TU_LOG2("  AUDIO get_req_entity: entity=%u, selector=%u, request=%u\r\n", request->bEntityID,
	        request->bControlSelector, request->bRequest);

	// Clock source requests
	if (request->bEntityID == UAC2_ENTITY_CLOCK) {
		if (request->bControlSelector == AUDIO20_CS_CTRL_SAM_FREQ && request->bRequest == AUDIO20_CS_REQ_CUR) {
			// Return current sample rate (44.1kHz)
			audio20_control_cur_4_t sample_rate = {.bCur = 44100};
			return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &sample_rate, sizeof(sample_rate));
		}
		else if (request->bControlSelector == AUDIO20_CS_CTRL_SAM_FREQ && request->bRequest == AUDIO20_CS_REQ_RANGE) {
			// Return supported sample rate range (only 44.1kHz)
			audio20_control_range_4_n_t(1) sample_rate_range = {};
			sample_rate_range.wNumSubRanges = 1;
			sample_rate_range.subrange[0].bMin = 44100;
			sample_rate_range.subrange[0].bMax = 44100;
			sample_rate_range.subrange[0].bRes = 0;
			return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &sample_rate_range,
			                                                  sizeof(sample_rate_range));
		}
		else if (request->bControlSelector == AUDIO20_CS_CTRL_CLK_VALID && request->bRequest == AUDIO20_CS_REQ_CUR) {
			// Clock is always valid
			audio20_control_cur_1_t clock_valid = {.bCur = 1};
			return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &clock_valid, sizeof(clock_valid));
		}
	}
	// Speaker feature unit requests (volume/mute)
	else if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
		if (request->bControlSelector == AUDIO20_FU_CTRL_MUTE && request->bRequest == AUDIO20_CS_REQ_CUR) {
			// Return mute state (not muted)
			audio20_control_cur_1_t mute = {.bCur = 0};
			return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &mute, sizeof(mute));
		}
		else if (request->bControlSelector == AUDIO20_FU_CTRL_VOLUME) {
			if (request->bRequest == AUDIO20_CS_REQ_RANGE) {
				// Volume range: -50dB to 0dB, 1dB steps
				audio20_control_range_2_n_t(1) volume_range = {};
				volume_range.wNumSubRanges = 1;
				volume_range.subrange[0].bMin = -50 * 256;
				volume_range.subrange[0].bMax = 0;
				volume_range.subrange[0].bRes = 256;
				return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &volume_range,
				                                                  sizeof(volume_range));
			}
			else if (request->bRequest == AUDIO20_CS_REQ_CUR) {
				// Current volume: 0dB (max)
				audio20_control_cur_2_t volume = {.bCur = 0};
				return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &volume, sizeof(volume));
			}
		}
	}
#if CFG_TUD_AUDIO_ENABLE_EP_IN
	// Microphone feature unit requests (volume/mute) - headset mode only
	else if (request->bEntityID == UAC2_ENTITY_MIC_FEATURE_UNIT) {
		if (request->bControlSelector == AUDIO20_FU_CTRL_MUTE && request->bRequest == AUDIO20_CS_REQ_CUR) {
			// Return mute state (not muted)
			audio20_control_cur_1_t mute = {.bCur = 0};
			return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &mute, sizeof(mute));
		}
		else if (request->bControlSelector == AUDIO20_FU_CTRL_VOLUME) {
			if (request->bRequest == AUDIO20_CS_REQ_RANGE) {
				// Volume range: -50dB to 0dB, 1dB steps
				audio20_control_range_2_n_t(1) volume_range = {};
				volume_range.wNumSubRanges = 1;
				volume_range.subrange[0].bMin = -50 * 256;
				volume_range.subrange[0].bMax = 0;
				volume_range.subrange[0].bRes = 256;
				return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &volume_range,
				                                                  sizeof(volume_range));
			}
			else if (request->bRequest == AUDIO20_CS_REQ_CUR) {
				// Current volume: 0dB (max)
				audio20_control_cur_2_t volume = {.bCur = 0};
				return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &volume, sizeof(volume));
			}
		}
	}
#endif // CFG_TUD_AUDIO_ENABLE_EP_IN

	CDBG_FMT("[UAC] UNHANDLED get_req entity=%u sel=%u req=%u\n", request->bEntityID,
	          request->bControlSelector, request->bRequest);
	return false;
}

} // extern "C"

#else // CFG_TUD_AUDIO == 0

// Stub implementations when audio is disabled
void usb_audio_init(void) {
}

void usb_audio_task(void) {
}

bool usb_audio_is_streaming(void) {
	return false;
}

void usb_audio_update_speaker_state(void) {
}

#endif // CFG_TUD_AUDIO
