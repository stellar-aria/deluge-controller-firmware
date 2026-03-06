/*
 * Deluge USB Controller - Descriptor Definitions
 * Full UAC2: Stereo speaker output + stereo microphone input
 * Alt 1: 16-bit 44.1kHz, Alt 2: 24-bit 44.1kHz
 */

#pragma once

//--------------------------------------------------------------------
// USB Audio mode selection
//--------------------------------------------------------------------
// USB_AUDIO_SPEAKER_ONLY = 0 (default): Stereo speaker + stereo microphone (headset).
//   Speaker OUT uses asynchronous sync with implicit feedback.
//   Spec-correct for fixed-clock devices (crystal-driven codec).
//   The mic IN endpoint serves as the implicit feedback source (USB 2.0 §5.12.4.2).
//   Linux and macOS support this natively. Windows requires the DelugeASIO driver
//   (contrib/asio-driver/) which handles implicit feedback via direct isochronous I/O.
//   Uses both RUSB1 ISO pipes: pipe 1 = speaker OUT, pipe 2 = mic IN.
//
// USB_AUDIO_SPEAKER_ONLY = 1: Speaker-only with explicit feedback endpoint (no mic).
//   Speaker OUT uses asynchronous sync with an explicit ISO feedback IN endpoint.
//   Compatible with Windows usbaudio2.sys class driver (no custom driver needed).
//   The device sends a 16.16 feedback value telling the host its exact sample rate.
//   Uses both RUSB1 ISO pipes: pipe 1 = speaker OUT, pipe 2 = feedback IN.
//   No microphone input in this mode.
//
// Override via CMake: -DUSB_AUDIO_SPEAKER_ONLY=1 for Windows class-driver mode.
#ifndef USB_AUDIO_SPEAKER_ONLY
#define USB_AUDIO_SPEAKER_ONLY 0
#endif

// Endpoint attribute helpers based on mode selection
// Both modes use asynchronous speaker OUT. Mode 0 has implicit feedback via mic IN.
// Mode 1 has explicit feedback EP (no mic).
#define USB_AUDIO_SPK_EP_SYNC TUSB_ISO_EP_ATT_ASYNCHRONOUS

#if !USB_AUDIO_SPEAKER_ONLY
#define USB_AUDIO_MIC_EP_USAGE TUSB_ISO_EP_ATT_IMPLICIT_FB
#endif

// Unit numbers for UAC2 entities
#define UAC2_ENTITY_CLOCK 0x04

// Speaker path (output from PC to Deluge DAC)
#define UAC2_ENTITY_SPK_INPUT_TERMINAL 0x01
#define UAC2_ENTITY_SPK_FEATURE_UNIT 0x02
#define UAC2_ENTITY_SPK_OUTPUT_TERMINAL 0x03

#if !USB_AUDIO_SPEAKER_ONLY
// Microphone path (input from Deluge ADC to PC) - only in headset mode
#define UAC2_ENTITY_MIC_INPUT_TERMINAL 0x11
#define UAC2_ENTITY_MIC_FEATURE_UNIT 0x12
#define UAC2_ENTITY_MIC_OUTPUT_TERMINAL 0x13
#endif

// Interface numbers (must match descriptor order)
// Audio first for Windows UAC2 driver compatibility
enum {
	ITF_NUM_AUDIO_CONTROL = 0,
	ITF_NUM_AUDIO_STREAMING_SPK,
#if !USB_AUDIO_SPEAKER_ONLY
	ITF_NUM_AUDIO_STREAMING_MIC,
#endif
	ITF_NUM_CDC,
	ITF_NUM_CDC_DATA,
	ITF_NUM_MIDI,
	ITF_NUM_MIDI_STREAMING,
	ITF_NUM_TOTAL
};

#if USB_AUDIO_SPEAKER_ONLY
// ============================================================================
// Speaker-only mode: async speaker with explicit feedback EP (no microphone)
// ============================================================================

// Audio Control Interface descriptor length (clock + speaker path only)
#define TUD_AUDIO_AC_DESC_LEN                                                                                          \
	(TUD_AUDIO20_DESC_CLK_SRC_LEN                                                                                      \
	 /* Speaker path */                                                                                                \
	 + TUD_AUDIO20_DESC_INPUT_TERM_LEN + TUD_AUDIO20_DESC_FEATURE_UNIT_LEN(2) + TUD_AUDIO20_DESC_OUTPUT_TERM_LEN)

// Per-alternate descriptor block length (STD_AS_INT + CS_AS_INT + TYPE_I_FORMAT + STD_AS_ISO_EP + CS_AS_ISO_EP + FB_EP)
#define TUD_AUDIO_ALT_DESC_LEN                                                                                         \
	(TUD_AUDIO20_DESC_STD_AS_LEN + TUD_AUDIO20_DESC_CS_AS_INT_LEN + TUD_AUDIO20_DESC_TYPE_I_FORMAT_LEN                 \
	 + TUD_AUDIO20_DESC_STD_AS_ISO_EP_LEN + TUD_AUDIO20_DESC_CS_AS_ISO_EP_LEN                                          \
	 + TUD_AUDIO20_DESC_STD_AS_ISO_FB_EP_LEN)

// Full audio descriptor length - speaker with feedback, alt0 (idle) + alt1 (16-bit) + alt2 (24-bit)
#define TUD_AUDIO_SPEAKER_STEREO_FB_DESC_LEN                                                                           \
	(TUD_AUDIO20_DESC_IAD_LEN + TUD_AUDIO20_DESC_STD_AC_LEN + TUD_AUDIO20_DESC_CS_AC_LEN + TUD_AUDIO_AC_DESC_LEN       \
	 /* Interface 1 (Speaker): Alt 0 (idle) + Alt 1 (16-bit + FB) + Alt 2 (24-bit + FB) */                            \
	 + TUD_AUDIO20_DESC_STD_AS_LEN + TUD_AUDIO_ALT_DESC_LEN + TUD_AUDIO_ALT_DESC_LEN)

// Speaker-only descriptor macro - stereo async speaker with explicit feedback EP
#define TUD_AUDIO_SPEAKER_STEREO_FB_DESCRIPTOR(_stridx, _epout, _epfb)                                                 \
	/* Standard Interface Association Descriptor (IAD) - 2 interfaces: AC + SPK AS */                                  \
	TUD_AUDIO20_DESC_IAD(/*_firstitf*/ ITF_NUM_AUDIO_CONTROL, /*_nitfs*/ 2, /*_stridx*/ 0x00),                         \
	    /* Standard AC Interface Descriptor(4.7.1) */                                                                  \
	    TUD_AUDIO20_DESC_STD_AC(/*_itfnum*/ ITF_NUM_AUDIO_CONTROL, /*_nEPs*/ 0x00, /*_stridx*/ _stridx),               \
	    /* Class-Specific AC Interface Header Descriptor(4.7.2) */                                                     \
	    TUD_AUDIO20_DESC_CS_AC(/*_bcdADC*/ 0x0200, /*_category*/ AUDIO20_FUNC_DESKTOP_SPEAKER,                         \
	                           /*_totallen*/ TUD_AUDIO_AC_DESC_LEN,                                                    \
	                           /*_ctrl*/ AUDIO20_CS_AS_INTERFACE_CTRL_LATENCY_POS),                                    \
	    /* Clock Source Descriptor(4.7.2.1) - Internal fixed clock (crystal) */                                        \
	    TUD_AUDIO20_DESC_CLK_SRC(/*_clkid*/ UAC2_ENTITY_CLOCK, /*_attr*/ AUDIO20_CLOCK_SOURCE_ATT_INT_FIX_CLK,         \
	                             /*_ctrl*/ 0x05, /*_assocTerm*/ 0x00,                                                  \
	                             /*_stridx*/ 0x00),                                                                    \
	    /* ===== Speaker path (USB to Deluge DAC) ===== */                                                             \
	    /* Input Terminal Descriptor(4.7.2.4) - USB streaming input */                                                 \
	    TUD_AUDIO20_DESC_INPUT_TERM(/*_termid*/ UAC2_ENTITY_SPK_INPUT_TERMINAL,                                        \
	                                /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING,                                       \
	                                /*_assocTerm*/ 0x00, /*_clkid*/ UAC2_ENTITY_CLOCK,                                 \
	                                /*_nchannelslogical*/ 0x02,                                                        \
	                                /*_channelcfg*/ AUDIO20_CHANNEL_CONFIG_NON_PREDEFINED,                             \
	                                /*_idxchannelnames*/ 0x00, /*_ctrl*/ 0x0000, /*_stridx*/ 0x00),                    \
	    /* Feature Unit Descriptor(4.7.2.8) - Volume/Mute control */                                                   \
	    TUD_AUDIO20_DESC_FEATURE_UNIT(                                                                                 \
	        /*_unitid*/ UAC2_ENTITY_SPK_FEATURE_UNIT, /*_srcid*/ UAC2_ENTITY_SPK_INPUT_TERMINAL, /*_stridx*/ 0x00,     \
	        /*_ctrlch0master*/                                                                                         \
	        (AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_MUTE_POS                                                     \
	         | AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_VOLUME_POS),                                               \
	        /*_ctrlch1*/                                                                                               \
	        (AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_MUTE_POS                                                     \
	         | AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_VOLUME_POS),                                               \
	        /*_ctrlch2*/                                                                                               \
	        (AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_MUTE_POS                                                     \
	         | AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_VOLUME_POS)),                                              \
	    /* Output Terminal Descriptor(4.7.2.5) - Speaker output */                                                     \
	    TUD_AUDIO20_DESC_OUTPUT_TERM(/*_termid*/ UAC2_ENTITY_SPK_OUTPUT_TERMINAL,                                      \
	                                 /*_termtype*/ AUDIO_TERM_TYPE_OUT_DESKTOP_SPEAKER,                                \
	                                 /*_assocTerm*/ 0x00,                                                              \
	                                 /*_srcid*/ UAC2_ENTITY_SPK_FEATURE_UNIT, /*_clkid*/ UAC2_ENTITY_CLOCK,            \
	                                 /*_ctrl*/ 0x0000, /*_stridx*/ 0x00),                                              \
	    /* ===== Interface 1: Speaker Streaming (USB OUT) with Feedback ===== */                                       \
	    /* Standard AS Interface Descriptor(4.9.1) - Alternate 0 (idle) */                                             \
	    TUD_AUDIO20_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)(ITF_NUM_AUDIO_STREAMING_SPK), /*_altset*/ 0x00,              \
	                                /*_nEPs*/ 0x00, /*_stridx*/ 0x00),                                                 \
	    /* --- Alternate 1: 16-bit stereo 44.1kHz + feedback EP --- */                                                 \
	    TUD_AUDIO20_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)(ITF_NUM_AUDIO_STREAMING_SPK), /*_altset*/ 0x01,              \
	                                /*_nEPs*/ 0x02, /*_stridx*/ 0x00),                                                 \
	    TUD_AUDIO20_DESC_CS_AS_INT(/*_termid*/ UAC2_ENTITY_SPK_INPUT_TERMINAL, /*_ctrl*/ AUDIO20_CTRL_NONE,            \
	                               /*_formattype*/ AUDIO20_FORMAT_TYPE_I, /*_formats*/ AUDIO20_DATA_FORMAT_TYPE_I_PCM, \
	                               /*_nchannelsphysical*/ 0x02,                                                        \
	                               /*_channelcfg*/ AUDIO20_CHANNEL_CONFIG_NON_PREDEFINED,                              \
	                               /*_stridx*/ 0x00),                                                                  \
	    TUD_AUDIO20_DESC_TYPE_I_FORMAT(/*_subslotsize*/ 0x02, /*_bitresolution*/ 16),                                  \
	    TUD_AUDIO20_DESC_STD_AS_ISO_EP(                                                                                \
	        /*_ep*/ _epout, /*_attr*/                                                                                  \
	        (uint8_t)((uint8_t)TUSB_XFER_ISOCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_ASYNCHRONOUS                           \
	                  | (uint8_t)TUSB_ISO_EP_ATT_DATA),                                                                \
        /*_maxEPsize*/ CFG_TUD_AUDIO_FUNC_1_EP_SZ_OUT_16, /*_interval*/ CFG_TUD_AUDIO_HS_BINTERVAL),               \
	    TUD_AUDIO20_DESC_CS_AS_ISO_EP(/*_attr*/ AUDIO20_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK,                      \
	                                  /*_ctrl*/ AUDIO20_CTRL_NONE,                                                     \
	                                  /*_lockdelayunit*/ AUDIO20_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC,           \
	                                  /*_lockdelay*/ 0x0001),                                                          \
	    TUD_AUDIO20_DESC_STD_AS_ISO_FB_EP(/*_ep*/ _epfb, /*_epsize*/ 4, /*_interval*/ CFG_TUD_AUDIO_HS_BINTERVAL),    \
	    /* --- Alternate 2: 24-bit stereo 44.1kHz + feedback EP --- */                                                 \
	    TUD_AUDIO20_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)(ITF_NUM_AUDIO_STREAMING_SPK), /*_altset*/ 0x02,              \
	                                /*_nEPs*/ 0x02, /*_stridx*/ 0x00),                                                 \
	    TUD_AUDIO20_DESC_CS_AS_INT(/*_termid*/ UAC2_ENTITY_SPK_INPUT_TERMINAL, /*_ctrl*/ AUDIO20_CTRL_NONE,            \
	                               /*_formattype*/ AUDIO20_FORMAT_TYPE_I, /*_formats*/ AUDIO20_DATA_FORMAT_TYPE_I_PCM, \
	                               /*_nchannelsphysical*/ 0x02,                                                        \
	                               /*_channelcfg*/ AUDIO20_CHANNEL_CONFIG_NON_PREDEFINED,                              \
	                               /*_stridx*/ 0x00),                                                                  \
	    TUD_AUDIO20_DESC_TYPE_I_FORMAT(/*_subslotsize*/ 0x03, /*_bitresolution*/ 24),                                  \
	    TUD_AUDIO20_DESC_STD_AS_ISO_EP(                                                                                \
	        /*_ep*/ _epout, /*_attr*/                                                                                  \
	        (uint8_t)((uint8_t)TUSB_XFER_ISOCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_ASYNCHRONOUS                           \
	                  | (uint8_t)TUSB_ISO_EP_ATT_DATA),                                                                \
        /*_maxEPsize*/ CFG_TUD_AUDIO_FUNC_1_EP_SZ_OUT_24, /*_interval*/ CFG_TUD_AUDIO_HS_BINTERVAL),               \
	    TUD_AUDIO20_DESC_CS_AS_ISO_EP(/*_attr*/ AUDIO20_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK,                      \
	                                  /*_ctrl*/ AUDIO20_CTRL_NONE,                                                     \
	                                  /*_lockdelayunit*/ AUDIO20_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC,           \
	                                  /*_lockdelay*/ 0x0001),                                                          \
	    TUD_AUDIO20_DESC_STD_AS_ISO_FB_EP(/*_ep*/ _epfb, /*_epsize*/ 4, /*_interval*/ CFG_TUD_AUDIO_HS_BINTERVAL)

#else // !USB_AUDIO_SPEAKER_ONLY
// ============================================================================
// Headset mode: async speaker + implicit-feedback microphone
// ============================================================================

// Audio Control Interface descriptor length (clock + speaker path + mic path)
#define TUD_AUDIO_AC_DESC_LEN                                                                                          \
	(TUD_AUDIO20_DESC_CLK_SRC_LEN                                                                                      \
	 /* Speaker path */                                                                                                \
	 + TUD_AUDIO20_DESC_INPUT_TERM_LEN + TUD_AUDIO20_DESC_FEATURE_UNIT_LEN(2) + TUD_AUDIO20_DESC_OUTPUT_TERM_LEN       \
	 /* Microphone path */                                                                                             \
	 + TUD_AUDIO20_DESC_INPUT_TERM_LEN + TUD_AUDIO20_DESC_FEATURE_UNIT_LEN(2) + TUD_AUDIO20_DESC_OUTPUT_TERM_LEN)

// Per-alternate descriptor block length (STD_AS_INT + CS_AS_INT + TYPE_I_FORMAT + STD_AS_ISO_EP + CS_AS_ISO_EP)
#define TUD_AUDIO_ALT_DESC_LEN                                                                                         \
	(TUD_AUDIO20_DESC_STD_AS_LEN + TUD_AUDIO20_DESC_CS_AS_INT_LEN + TUD_AUDIO20_DESC_TYPE_I_FORMAT_LEN                 \
	 + TUD_AUDIO20_DESC_STD_AS_ISO_EP_LEN + TUD_AUDIO20_DESC_CS_AS_ISO_EP_LEN)

// Full audio descriptor length - speaker + microphone, each with alt0 (idle) + alt1 (16-bit) + alt2 (24-bit)
#define TUD_AUDIO_HEADSET_STEREO_DESC_LEN                                                                              \
	(TUD_AUDIO20_DESC_IAD_LEN + TUD_AUDIO20_DESC_STD_AC_LEN + TUD_AUDIO20_DESC_CS_AC_LEN + TUD_AUDIO_AC_DESC_LEN       \
	 /* Interface 1 (Speaker): Alt 0 (idle) + Alt 1 (16-bit) + Alt 2 (24-bit) */                                      \
	 + TUD_AUDIO20_DESC_STD_AS_LEN + TUD_AUDIO_ALT_DESC_LEN + TUD_AUDIO_ALT_DESC_LEN                                  \
	 /* Interface 2 (Microphone): Alt 0 (idle) + Alt 1 (16-bit) + Alt 2 (24-bit) */                                   \
	 + TUD_AUDIO20_DESC_STD_AS_LEN + TUD_AUDIO_ALT_DESC_LEN + TUD_AUDIO_ALT_DESC_LEN)

// Full headset descriptor macro - stereo speaker + stereo microphone
#define TUD_AUDIO_HEADSET_STEREO_DESCRIPTOR(_stridx, _epout, _epin)                                                    \
	/* Standard Interface Association Descriptor (IAD) - 3 interfaces: AC + SPK AS + MIC AS */                         \
	TUD_AUDIO20_DESC_IAD(/*_firstitf*/ ITF_NUM_AUDIO_CONTROL, /*_nitfs*/ 3, /*_stridx*/ 0x00),                         \
	    /* Standard AC Interface Descriptor(4.7.1) */                                                                  \
	    TUD_AUDIO20_DESC_STD_AC(/*_itfnum*/ ITF_NUM_AUDIO_CONTROL, /*_nEPs*/ 0x00, /*_stridx*/ _stridx),               \
	    /* Class-Specific AC Interface Header Descriptor(4.7.2) */                                                     \
	    TUD_AUDIO20_DESC_CS_AC(/*_bcdADC*/ 0x0200, /*_category*/ AUDIO20_FUNC_HEADSET,                                 \
	                           /*_totallen*/ TUD_AUDIO_AC_DESC_LEN,                                                    \
	                           /*_ctrl*/ AUDIO20_CS_AS_INTERFACE_CTRL_LATENCY_POS),                                    \
	    /* Clock Source Descriptor(4.7.2.1) - Internal fixed clock (crystal) */                                       \
	    TUD_AUDIO20_DESC_CLK_SRC(/*_clkid*/ UAC2_ENTITY_CLOCK, /*_attr*/ AUDIO20_CLOCK_SOURCE_ATT_INT_FIX_CLK,         \
	                             /*_ctrl*/ 0x05, /*_assocTerm*/ 0x00,                                                  \
	                             /*_stridx*/ 0x00),                                                                    \
	    /* ===== Speaker path (USB to Deluge DAC) ===== */                                                             \
	    /* Input Terminal Descriptor(4.7.2.4) - USB streaming input */                                                 \
	    TUD_AUDIO20_DESC_INPUT_TERM(/*_termid*/ UAC2_ENTITY_SPK_INPUT_TERMINAL,                                        \
	                                /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING,                                       \
	                                /*_assocTerm*/ UAC2_ENTITY_MIC_OUTPUT_TERMINAL, /*_clkid*/ UAC2_ENTITY_CLOCK,      \
	                                /*_nchannelslogical*/ 0x02,                                                        \
	                                /*_channelcfg*/ AUDIO20_CHANNEL_CONFIG_NON_PREDEFINED,                             \
	                                /*_idxchannelnames*/ 0x00, /*_ctrl*/ 0x0000, /*_stridx*/ 0x00),                    \
	    /* Feature Unit Descriptor(4.7.2.8) - Volume/Mute control */                                                   \
	    TUD_AUDIO20_DESC_FEATURE_UNIT(                                                                                 \
	        /*_unitid*/ UAC2_ENTITY_SPK_FEATURE_UNIT, /*_srcid*/ UAC2_ENTITY_SPK_INPUT_TERMINAL, /*_stridx*/ 0x00,     \
	        /*_ctrlch0master*/                                                                                         \
	        (AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_MUTE_POS                                                     \
	         | AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_VOLUME_POS),                                               \
	        /*_ctrlch1*/                                                                                               \
	        (AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_MUTE_POS                                                     \
	         | AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_VOLUME_POS),                                               \
	        /*_ctrlch2*/                                                                                               \
	        (AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_MUTE_POS                                                     \
	         | AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_VOLUME_POS)),                                              \
	    /* Output Terminal Descriptor(4.7.2.5) - Headphones/Speaker output */                                          \
	    TUD_AUDIO20_DESC_OUTPUT_TERM(/*_termid*/ UAC2_ENTITY_SPK_OUTPUT_TERMINAL,                                      \
	                                 /*_termtype*/ AUDIO_TERM_TYPE_OUT_HEADPHONES,                                     \
	                                 /*_assocTerm*/ UAC2_ENTITY_MIC_INPUT_TERMINAL,                                    \
	                                 /*_srcid*/ UAC2_ENTITY_SPK_FEATURE_UNIT, /*_clkid*/ UAC2_ENTITY_CLOCK,            \
	                                 /*_ctrl*/ 0x0000, /*_stridx*/ 0x00),                                              \
	    /* ===== Microphone path (Deluge ADC to USB) ===== */                                                          \
	    /* Input Terminal Descriptor(4.7.2.4) - Microphone/Line input */                                               \
	    TUD_AUDIO20_DESC_INPUT_TERM(/*_termid*/ UAC2_ENTITY_MIC_INPUT_TERMINAL,                                        \
	                                /*_termtype*/ AUDIO_TERM_TYPE_IN_GENERIC_MIC,                                      \
	                                /*_assocTerm*/ UAC2_ENTITY_SPK_OUTPUT_TERMINAL, /*_clkid*/ UAC2_ENTITY_CLOCK,      \
	                                /*_nchannelslogical*/ 0x02,                                                        \
	                                /*_channelcfg*/ AUDIO20_CHANNEL_CONFIG_NON_PREDEFINED,                             \
	                                /*_idxchannelnames*/ 0x00, /*_ctrl*/ 0x0000, /*_stridx*/ 0x00),                    \
	    /* Feature Unit Descriptor(4.7.2.8) - Volume/Mute control */                                                   \
	    TUD_AUDIO20_DESC_FEATURE_UNIT(                                                                                 \
	        /*_unitid*/ UAC2_ENTITY_MIC_FEATURE_UNIT, /*_srcid*/ UAC2_ENTITY_MIC_INPUT_TERMINAL, /*_stridx*/ 0x00,     \
	        /*_ctrlch0master*/                                                                                         \
	        (AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_MUTE_POS                                                     \
	         | AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_VOLUME_POS),                                               \
	        /*_ctrlch1*/                                                                                               \
	        (AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_MUTE_POS                                                     \
	         | AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_VOLUME_POS),                                               \
	        /*_ctrlch2*/                                                                                               \
	        (AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_MUTE_POS                                                     \
	         | AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_VOLUME_POS)),                                              \
	    /* Output Terminal Descriptor(4.7.2.5) - USB streaming output */                                               \
	    TUD_AUDIO20_DESC_OUTPUT_TERM(/*_termid*/ UAC2_ENTITY_MIC_OUTPUT_TERMINAL,                                      \
	                                 /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING,                                      \
	                                 /*_assocTerm*/ UAC2_ENTITY_SPK_INPUT_TERMINAL,                                    \
	                                 /*_srcid*/ UAC2_ENTITY_MIC_FEATURE_UNIT, /*_clkid*/ UAC2_ENTITY_CLOCK,            \
	                                 /*_ctrl*/ 0x0000, /*_stridx*/ 0x00),                                              \
	    /* ===== Interface 1: Speaker Streaming (USB OUT) ===== */                                                     \
	    /* Standard AS Interface Descriptor(4.9.1) - Alternate 0 (idle) */                                             \
	    TUD_AUDIO20_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)(ITF_NUM_AUDIO_STREAMING_SPK), /*_altset*/ 0x00,              \
	                                /*_nEPs*/ 0x00, /*_stridx*/ 0x00),                                                 \
	    /* --- Alternate 1: 16-bit stereo 44.1kHz --- */                                                               \
	    TUD_AUDIO20_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)(ITF_NUM_AUDIO_STREAMING_SPK), /*_altset*/ 0x01,              \
	                                /*_nEPs*/ 0x01, /*_stridx*/ 0x00),                                                 \
	    TUD_AUDIO20_DESC_CS_AS_INT(/*_termid*/ UAC2_ENTITY_SPK_INPUT_TERMINAL, /*_ctrl*/ AUDIO20_CTRL_NONE,            \
	                               /*_formattype*/ AUDIO20_FORMAT_TYPE_I, /*_formats*/ AUDIO20_DATA_FORMAT_TYPE_I_PCM, \
	                               /*_nchannelsphysical*/ 0x02,                                                        \
	                               /*_channelcfg*/ AUDIO20_CHANNEL_CONFIG_NON_PREDEFINED,                              \
	                               /*_stridx*/ 0x00),                                                                  \
	    TUD_AUDIO20_DESC_TYPE_I_FORMAT(/*_subslotsize*/ 0x02, /*_bitresolution*/ 16),                                  \
	    TUD_AUDIO20_DESC_STD_AS_ISO_EP(                                                                                \
	        /*_ep*/ _epout, /*_attr*/                                                                                  \
	        (uint8_t)((uint8_t)TUSB_XFER_ISOCHRONOUS | (uint8_t)USB_AUDIO_SPK_EP_SYNC                                  \
	                  | (uint8_t)TUSB_ISO_EP_ATT_DATA),                                                                \
        /*_maxEPsize*/ CFG_TUD_AUDIO_FUNC_1_EP_SZ_OUT_16, /*_interval*/ CFG_TUD_AUDIO_HS_BINTERVAL),               \
	    TUD_AUDIO20_DESC_CS_AS_ISO_EP(/*_attr*/ AUDIO20_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK,                      \
	                                  /*_ctrl*/ AUDIO20_CTRL_NONE,                                                     \
	                                  /*_lockdelayunit*/ AUDIO20_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC,           \
	                                  /*_lockdelay*/ 0x0001),                                                          \
	    /* --- Alternate 2: 24-bit stereo 44.1kHz --- */                                                               \
	    TUD_AUDIO20_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)(ITF_NUM_AUDIO_STREAMING_SPK), /*_altset*/ 0x02,              \
	                                /*_nEPs*/ 0x01, /*_stridx*/ 0x00),                                                 \
	    TUD_AUDIO20_DESC_CS_AS_INT(/*_termid*/ UAC2_ENTITY_SPK_INPUT_TERMINAL, /*_ctrl*/ AUDIO20_CTRL_NONE,            \
	                               /*_formattype*/ AUDIO20_FORMAT_TYPE_I, /*_formats*/ AUDIO20_DATA_FORMAT_TYPE_I_PCM, \
	                               /*_nchannelsphysical*/ 0x02,                                                        \
	                               /*_channelcfg*/ AUDIO20_CHANNEL_CONFIG_NON_PREDEFINED,                              \
	                               /*_stridx*/ 0x00),                                                                  \
	    TUD_AUDIO20_DESC_TYPE_I_FORMAT(/*_subslotsize*/ 0x03, /*_bitresolution*/ 24),                                  \
	    TUD_AUDIO20_DESC_STD_AS_ISO_EP(                                                                                \
	        /*_ep*/ _epout, /*_attr*/                                                                                  \
	        (uint8_t)((uint8_t)TUSB_XFER_ISOCHRONOUS | (uint8_t)USB_AUDIO_SPK_EP_SYNC                                  \
	                  | (uint8_t)TUSB_ISO_EP_ATT_DATA),                                                                \
        /*_maxEPsize*/ CFG_TUD_AUDIO_FUNC_1_EP_SZ_OUT_24, /*_interval*/ CFG_TUD_AUDIO_HS_BINTERVAL),               \
	    TUD_AUDIO20_DESC_CS_AS_ISO_EP(/*_attr*/ AUDIO20_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK,                      \
	                                  /*_ctrl*/ AUDIO20_CTRL_NONE,                                                     \
	                                  /*_lockdelayunit*/ AUDIO20_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC,           \
	                                  /*_lockdelay*/ 0x0001),                                                          \
	    /* ===== Interface 2: Microphone Streaming (USB IN) ===== */                                                   \
	    /* Standard AS Interface Descriptor(4.9.1) - Alternate 0 (idle) */                                             \
	    TUD_AUDIO20_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)(ITF_NUM_AUDIO_STREAMING_MIC), /*_altset*/ 0x00,              \
	                                /*_nEPs*/ 0x00, /*_stridx*/ 0x00),                                                 \
	    /* --- Alternate 1: 16-bit stereo 44.1kHz --- */                                                               \
	    TUD_AUDIO20_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)(ITF_NUM_AUDIO_STREAMING_MIC), /*_altset*/ 0x01,              \
	                                /*_nEPs*/ 0x01, /*_stridx*/ 0x00),                                                 \
	    TUD_AUDIO20_DESC_CS_AS_INT(/*_termid*/ UAC2_ENTITY_MIC_OUTPUT_TERMINAL, /*_ctrl*/ AUDIO20_CTRL_NONE,           \
	                               /*_formattype*/ AUDIO20_FORMAT_TYPE_I, /*_formats*/ AUDIO20_DATA_FORMAT_TYPE_I_PCM, \
	                               /*_nchannelsphysical*/ 0x02,                                                        \
	                               /*_channelcfg*/ AUDIO20_CHANNEL_CONFIG_NON_PREDEFINED,                              \
	                               /*_stridx*/ 0x00),                                                                  \
	    TUD_AUDIO20_DESC_TYPE_I_FORMAT(/*_subslotsize*/ 0x02, /*_bitresolution*/ 16),                                  \
	    TUD_AUDIO20_DESC_STD_AS_ISO_EP(                                                                                \
	        /*_ep*/ _epin, /*_attr*/                                                                                   \
	        (uint8_t)((uint8_t)TUSB_XFER_ISOCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_ASYNCHRONOUS                           \
	                  | (uint8_t)USB_AUDIO_MIC_EP_USAGE),                                                              \
        /*_maxEPsize*/ CFG_TUD_AUDIO_FUNC_1_EP_SZ_IN_16, /*_interval*/ CFG_TUD_AUDIO_HS_BINTERVAL),                \
	    TUD_AUDIO20_DESC_CS_AS_ISO_EP(/*_attr*/ AUDIO20_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK,                      \
	                                  /*_ctrl*/ AUDIO20_CTRL_NONE,                                                     \
	                                  /*_lockdelayunit*/ AUDIO20_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC,           \
	                                  /*_lockdelay*/ 0x0001),                                                          \
	    /* --- Alternate 2: 24-bit stereo 44.1kHz --- */                                                               \
	    TUD_AUDIO20_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)(ITF_NUM_AUDIO_STREAMING_MIC), /*_altset*/ 0x02,              \
	                                /*_nEPs*/ 0x01, /*_stridx*/ 0x00),                                                 \
	    TUD_AUDIO20_DESC_CS_AS_INT(/*_termid*/ UAC2_ENTITY_MIC_OUTPUT_TERMINAL, /*_ctrl*/ AUDIO20_CTRL_NONE,           \
	                               /*_formattype*/ AUDIO20_FORMAT_TYPE_I, /*_formats*/ AUDIO20_DATA_FORMAT_TYPE_I_PCM, \
	                               /*_nchannelsphysical*/ 0x02,                                                        \
	                               /*_channelcfg*/ AUDIO20_CHANNEL_CONFIG_NON_PREDEFINED,                              \
	                               /*_stridx*/ 0x00),                                                                  \
	    TUD_AUDIO20_DESC_TYPE_I_FORMAT(/*_subslotsize*/ 0x03, /*_bitresolution*/ 24),                                  \
	    TUD_AUDIO20_DESC_STD_AS_ISO_EP(                                                                                \
	        /*_ep*/ _epin, /*_attr*/                                                                                   \
	        (uint8_t)((uint8_t)TUSB_XFER_ISOCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_ASYNCHRONOUS                           \
	                  | (uint8_t)USB_AUDIO_MIC_EP_USAGE),                                                              \
        /*_maxEPsize*/ CFG_TUD_AUDIO_FUNC_1_EP_SZ_IN_24, /*_interval*/ CFG_TUD_AUDIO_HS_BINTERVAL),                \
	    TUD_AUDIO20_DESC_CS_AS_ISO_EP(/*_attr*/ AUDIO20_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK,                      \
	                                  /*_ctrl*/ AUDIO20_CTRL_NONE,                                                     \
	                                  /*_lockdelayunit*/ AUDIO20_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC,           \
	                                  /*_lockdelay*/ 0x0001)

#endif // USB_AUDIO_SPEAKER_ONLY

