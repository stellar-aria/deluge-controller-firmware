//! USB Audio Class 2.0 (UAC2) device-side driver for the Deluge.
//!
//! ## Quick start
//!
//! ```rust,no_run
//! let (mut audio, ep_out) = AudioClass::new(&mut builder, 288);
//! builder.handler(&mut audio);
//! let usb = builder.build();
//! ```

use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::descriptor::{SynchronizationType, UsageType};
use embassy_usb::driver::Driver;
use embassy_usb::types::InterfaceNumber;
use embassy_usb::{Builder, Handler};

const USB_CLASS_AUDIO: u8 = 0x01;
const USB_SUBCLASS_AUDIO_CONTROL: u8 = 0x01;
const USB_SUBCLASS_AUDIO_STREAMING: u8 = 0x02;
const USB_PROTOCOL_IP_02_00: u8 = 0x20;

const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;
const AC_HEADER: u8 = 0x01;
const AC_INPUT_TERM: u8 = 0x02;
const AC_OUTPUT_TERM: u8 = 0x03;
const AC_CLOCK_SOURCE: u8 = 0x0A;
const AS_GENERAL: u8 = 0x01;
const AS_FORMAT_TYPE: u8 = 0x02;

const CLOCK_SOURCE_ID: u8 = 0x10;
const INPUT_TERM_ID: u8 = 0x11;
const OUTPUT_TERM_ID: u8 = 0x12;

const UAC2_SET_CUR: u8 = 0x01;
const UAC2_GET_CUR: u8 = 0x81;
const SAM_FREQ_CS: u8 = 0x01;

const TERM_USB_STREAMING: u16 = 0x0101;
const TERM_SPEAKER: u16 = 0x0301;

pub const SAMPLE_RATE_44100: u32 = 44_100;
pub const SAMPLE_RATE_48000: u32 = 48_000;

/// Handles UAC2 SET_CUR / GET_CUR for sample-rate negotiation.
///
/// Create via [`AudioClass::new`], register with `builder.handler()`.
pub struct AudioClass {
    ac_iface: InterfaceNumber,
    as_iface: InterfaceNumber,
    /// Current sample rate in Hz (44100 or 48000).
    pub sample_rate: u32,
    /// True when the host selected alternate setting 1 (streaming active).
    pub streaming_active: bool,
}

impl AudioClass {
    /// Allocate USB interfaces and an ISO OUT endpoint for the UAC2 speaker.
    ///
    /// `iso_packet_size` — max bytes per isochronous packet.  288 covers
    /// both 44.1 kHz (264 B) and 48 kHz (288 B) at stereo 24-bit.
    ///
    /// Returns `(handler, ep_out)`.
    pub fn new<'d, D: Driver<'d>>(
        builder: &mut Builder<'d, D>,
        iso_packet_size: u16,
    ) -> (Self, D::EndpointOut) {
        let mut func = builder.function(USB_CLASS_AUDIO, 0x00, USB_PROTOCOL_IP_02_00);

        // ── AudioControl interface ──────────────────────────────────────────
        let mut ac_ib = func.interface();
        let ac_iface = ac_ib.interface_number();
        let mut ac_alt = ac_ib.alt_setting(
            USB_CLASS_AUDIO,
            USB_SUBCLASS_AUDIO_CONTROL,
            USB_PROTOCOL_IP_02_00,
            None,
        );
        // CS AC Header (wTotalLength = 9+8+17+12 = 46)
        ac_alt.descriptor(CS_INTERFACE, &[AC_HEADER, 0x00, 0x02, 0x08, 46, 0x00, 0x00]);
        // Clock Source (entity 0x10, internal, SAM_FREQ read-only)
        ac_alt.descriptor(
            CS_INTERFACE,
            &[AC_CLOCK_SOURCE, CLOCK_SOURCE_ID, 0b01, 0b01, 0x00, 0x00],
        );
        // Input Terminal: USB streaming (entity 0x11)
        ac_alt.descriptor(
            CS_INTERFACE,
            &[
                AC_INPUT_TERM,
                INPUT_TERM_ID,
                (TERM_USB_STREAMING & 0xFF) as u8,
                (TERM_USB_STREAMING >> 8) as u8,
                CLOCK_SOURCE_ID,
                0x02, // bNrChannels = 2
                0x03,
                0x00,
                0x00,
                0x00, // bmChannelConfig: FL + FR
                0x00,
                0x00,
                0x00,
                0x00,
            ],
        );
        // Output Terminal: generic speaker (entity 0x12)
        ac_alt.descriptor(
            CS_INTERFACE,
            &[
                AC_OUTPUT_TERM,
                OUTPUT_TERM_ID,
                (TERM_SPEAKER & 0xFF) as u8,
                (TERM_SPEAKER >> 8) as u8,
                INPUT_TERM_ID,
                CLOCK_SOURCE_ID,
                0x00,
                0x00,
                0x00,
            ],
        );

        // ── AudioStreaming interface ─────────────────────────────────────────
        let mut as_ib = func.interface();
        let as_iface = as_ib.interface_number();
        // Alt 0: zero-bandwidth (required by UAC2 spec)
        let _ = as_ib.alt_setting(
            USB_CLASS_AUDIO,
            USB_SUBCLASS_AUDIO_STREAMING,
            USB_PROTOCOL_IP_02_00,
            None,
        );
        // Alt 1: operational (ISO OUT, 2ch 24-bit PCM)
        let mut as_alt1 = as_ib.alt_setting(
            USB_CLASS_AUDIO,
            USB_SUBCLASS_AUDIO_STREAMING,
            USB_PROTOCOL_IP_02_00,
            None,
        );
        // CS AS General
        as_alt1.descriptor(
            CS_INTERFACE,
            &[
                AS_GENERAL,
                INPUT_TERM_ID,
                0x00,
                0x01,
                0x01,
                0x00,
                0x00,
                0x00, // bmFormats: PCM
                0x02,
                0x03,
                0x00,
                0x00,
                0x00, // bNrChannels + bmChannelConfig
                0x00,
            ],
        );
        // Format Type I (24-bit, 3 bytes/sample)
        as_alt1.descriptor(CS_INTERFACE, &[AS_FORMAT_TYPE, 0x01, 0x03, 0x18]);
        // Isochronous OUT data endpoint (synchronous — host provides clock)
        let ep_out = as_alt1.endpoint_isochronous_out(
            None,
            iso_packet_size,
            1,
            SynchronizationType::Synchronous,
            UsageType::DataEndpoint,
            &[],
        );
        // CS Endpoint General
        as_alt1.descriptor(CS_ENDPOINT, &[0x01, 0x00, 0x00, 0x00, 0x00, 0x00]);

        (
            Self {
                ac_iface,
                as_iface,
                sample_rate: SAMPLE_RATE_44100,
                streaming_active: false,
            },
            ep_out,
        )
    }

    /// Current sample rate in Hz.
    pub fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    /// True when the host has selected the active streaming alternate setting.
    pub fn is_streaming(&self) -> bool {
        self.streaming_active
    }
}

impl Handler for AudioClass {
    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        // UAC2 SET_CUR — clock source SAM_FREQ_CONTROL
        if req.request_type == RequestType::Class
            && req.recipient == Recipient::Interface
            && req.request == UAC2_SET_CUR
            && (req.index >> 8) as u8 == CLOCK_SOURCE_ID
            && (req.value >> 8) as u8 == SAM_FREQ_CS
        {
            if data.len() >= 4 {
                let hz = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                if hz == SAMPLE_RATE_44100 || hz == SAMPLE_RATE_48000 {
                    self.sample_rate = hz;
                }
            }
            return Some(OutResponse::Accepted);
        }
        None
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        // UAC2 GET_CUR — clock source SAM_FREQ_CONTROL
        if req.request_type == RequestType::Class
            && req.recipient == Recipient::Interface
            && req.request == UAC2_GET_CUR
            && (req.index >> 8) as u8 == CLOCK_SOURCE_ID
            && (req.value >> 8) as u8 == SAM_FREQ_CS
        {
            let bytes = self.sample_rate.to_le_bytes();
            let len = buf.len().min(4);
            buf[..len].copy_from_slice(&bytes[..len]);
            return Some(InResponse::Accepted(&buf[..len]));
        }
        None
    }

    fn set_alternate_setting(&mut self, iface: InterfaceNumber, alternate: u8) {
        if iface == self.as_iface {
            self.streaming_active = alternate == 1;
        }
    }

    fn reset(&mut self) {
        self.streaming_active = false;
    }
}
