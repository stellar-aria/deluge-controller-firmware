#include "usb_descriptors.h"
#include "class/cdc/cdc.h"
#include "class/midi/midi.h"
#include "tusb.h"
#include "tusb_config.h"

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device = {.bLength = sizeof(tusb_desc_device_t),
                                        .bDescriptorType = TUSB_DESC_DEVICE,
                                        .bcdUSB = 0x0200,
                                        .bDeviceClass = TUSB_CLASS_MISC,
                                        .bDeviceSubClass = MISC_SUBCLASS_COMMON,
                                        .bDeviceProtocol = MISC_PROTOCOL_IAD,
                                        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

                                        .idVendor = 0x16D0,  // MCS Electronics (for open source projects)
                                        .idProduct = 0x0EDA, // PID for Deluge Controller
                                        .bcdDevice = 0x0100,

                                        .iManufacturer = 0x01,
                                        .iProduct = 0x02,
                                        .iSerialNumber = 0x03,

                                        .bNumConfigurations = 0x01};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const* tud_descriptor_device_cb(void) {
	return (uint8_t const*)&desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

// Composite device: Audio + CDC + MIDI
// Mode 0: Full UAC2 headset (stereo speaker + stereo mic), async + implicit feedback
// Mode 1: Speaker-only UAC2 (stereo speaker + explicit feedback EP, no mic)
// MIDI needs an extra 8-byte IAD since TUD_MIDI_DESCRIPTOR doesn't include one,
// but the device declares IAD protocol (MISC_PROTOCOL_IAD) which requires all functions to have an IAD.
#define TUD_MIDI_IAD_LEN 8

#if USB_AUDIO_SPEAKER_ONLY
#define CFG_TUD_AUDIO_FUNC_1_TOTAL_DESC_LEN TUD_AUDIO_SPEAKER_STEREO_FB_DESC_LEN
#else
#define CFG_TUD_AUDIO_FUNC_1_TOTAL_DESC_LEN TUD_AUDIO_HEADSET_STEREO_DESC_LEN
#endif

#define CONFIG_TOTAL_LEN                                                                                               \
	(TUD_CONFIG_DESC_LEN + CFG_TUD_AUDIO_FUNC_1_TOTAL_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MIDI_IAD_LEN                  \
	 + TUD_MIDI_DESC_LEN)

// CDC endpoints
#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT 0x02
#define EPNUM_CDC_IN 0x82

// Audio endpoints
#define EPNUM_AUDIO_OUT 0x03 // Speaker output (PC to Deluge)

#if USB_AUDIO_SPEAKER_ONLY
// Speaker-only: feedback EP replaces mic on the same ISO pipe (pipe 2)
#define EPNUM_AUDIO_FB 0x83  // Explicit feedback IN (4-byte 16.16 rate)
#else
// Headset: mic IN also serves as implicit feedback source
#define EPNUM_AUDIO_IN 0x83  // Microphone input (Deluge to PC)
#endif

// MIDI endpoints - 512 bytes for High-Speed BULK
#define EPNUM_MIDI_OUT 0x05
#define EPNUM_MIDI_IN 0x85

uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 500),

#if USB_AUDIO_SPEAKER_ONLY
    // Audio Interface - UAC2 speaker with explicit feedback EP (no microphone)
    TUD_AUDIO_SPEAKER_STEREO_FB_DESCRIPTOR(4, EPNUM_AUDIO_OUT, EPNUM_AUDIO_FB),
#else
    // Audio Interface - UAC2 headset (stereo speaker + stereo microphone)
    TUD_AUDIO_HEADSET_STEREO_DESCRIPTOR(4, EPNUM_AUDIO_OUT, EPNUM_AUDIO_IN),
#endif

    // CDC Interface - Serial port
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 7, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, CFG_TUD_CDC_EP_BUFSIZE),

    // MIDI IAD - required because device uses IAD protocol and every function needs an IAD
    8, TUSB_DESC_INTERFACE_ASSOCIATION, ITF_NUM_MIDI, 2, TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_MIDI_STREAMING,
    AUDIO_FUNC_PROTOCOL_CODE_UNDEF, 0,

    // MIDI Interface - 512 bytes for High-Speed BULK
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 8, EPNUM_MIDI_OUT, EPNUM_MIDI_IN, 512),
};

// Compile-time check: descriptor array size must match computed total length
_Static_assert(sizeof(desc_configuration) == CONFIG_TOTAL_LEN,
    "desc_configuration size mismatch with CONFIG_TOTAL_LEN");

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
	(void)index; // for multiple configurations
	return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
char const* string_desc_arr[] = {
    (const char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
    "Synthstrom Audible",       // 1: Manufacturer
    "Deluge USB Controller",    // 2: Product
    "123456",                   // 3: Serials, should use chip ID
    "Deluge Audio",             // 4: Audio Interface
    "Deluge Speaker",           // 5: Audio Speaker Interface
    "Deluge Microphone",        // 6: Audio Microphone Interface (unused in mode 1)
    "Deluge Control Port",      // 7: CDC Interface
    "Deluge MIDI",              // 8: MIDI Interface
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
	(void)langid;

	uint8_t chr_count;

	if (index == 0) {
		memcpy(&_desc_str[1], string_desc_arr[0], 2);
		chr_count = 1;
	}
	else {
		// Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
		// https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

		if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
			return NULL;

		const char* str = string_desc_arr[index];

		// Cap at max char
		chr_count = (uint8_t)strlen(str);
		if (chr_count > 31)
			chr_count = 31;

		// Convert ASCII string into UTF-16
		for (uint8_t i = 0; i < chr_count; i++) {
			_desc_str[1 + i] = str[i];
		}
	}

	// first byte is length (including header), second byte is string type
	_desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

	return _desc_str;
}
