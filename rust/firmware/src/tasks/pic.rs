use log::{debug, info};

use deluge_bsp::oled;
use deluge_bsp::pic;
use crate::events::{EVENT_CHANNEL, HardwareEvent};
use crate::pads::{pad_toggle, pad_set_all, pad_invert_all};

/// PIC32 event dispatcher — input handling and LED feedback.
///
/// | Event                  | Action                                              |
/// |------------------------|-----------------------------------------------------|
/// | `PadPress { id }`         | Toggle pad; signal OLED redraw; log (x, y)          |
/// | `ButtonPress BACK`        | Clear all pads + LEDs; signal redraw                |
/// | `ButtonPress FILL`        | Fill all pads + LEDs; signal redraw                 |
/// | `ButtonPress SELECT`      | Invert all pads; signal redraw                      |
/// | `ButtonPress other`       | Light indicator LED while held                      |
/// | `ButtonRelease`           | Extinguish indicator LED                            |
/// | `EncoderDelta { id:2/3 }` | Drive LowerGold / UpperGold indicator ring          |
/// | `EncoderDelta { other }`  | Logged to RTT                                       |
/// | `OledSelected`            | Forward to [`pic::notify_oled_selected()`]          |
/// | `OledDeselected`          | Forward to [`pic::notify_oled_deselected()`]        |
#[embassy_executor::task]
pub(crate) async fn pic_task() {
    info!("PIC: init (31 250 → 200 000 baud)");
    pic::init().await;
    info!("PIC: ready");

    let mut parser = pic::Parser::new();
    debug!("pic_task: entering main loop");

    loop {
        let byte = rza1::uart::read_byte(pic::UART_CH).await;
        let Some(event) = parser.push(byte) else {
            continue;
        };

        match event {
            // ---- Pad toggle ------------------------------------------------
            pic::Event::PadPress { id } => {
                let lit = pad_toggle(id);
                let (col, row) = pic::pad_coords(id);
                info!(
                    "pad {} ({},{}) → {}",
                    id,
                    col,
                    row,
                    if lit { "on" } else { "off" }
                );
                oled::notify_redraw();
                let _ = EVENT_CHANNEL.try_send(HardwareEvent::PadPressed { col, row });
            }
            pic::Event::PadRelease { id } => {
                let (col, row) = pic::pad_coords(id);
                let _ = EVENT_CHANNEL.try_send(HardwareEvent::PadReleased { col, row });
            }

            // ---- Buttons ---------------------------------------------------
            pic::Event::ButtonPress { id } => {
                info!("button {} press", id);
                let _ = EVENT_CHANNEL.try_send(HardwareEvent::ButtonPressed { id: id + 144 });
                pic::led_on(id).await;
                match id {
                    pic::button::BACK => {
                        pad_set_all(false);
                        for led in 0..36u8 {
                            pic::led_off(led).await;
                        }
                        oled::notify_redraw();
                    }
                    pic::button::FILL => {
                        pad_set_all(true);
                        for led in 0..36u8 {
                            pic::led_on(led).await;
                        }
                        oled::notify_redraw();
                    }
                    pic::button::SELECT => {
                        pad_invert_all();
                        oled::notify_redraw();
                    }
                    _ => {}
                }
            }
            pic::Event::ButtonRelease { id } => {
                let _ = EVENT_CHANNEL.try_send(HardwareEvent::ButtonReleased { id: id + 144 });
                pic::led_off(id).await;
            }

            // ---- Encoders --------------------------------------------------
            // Encoders are read from GPIO by encoder_task; PIC does not send
            // encoder bytes.  This arm is a fallback in case the PIC firmware
            // ever adds encoder reporting.
            pic::Event::EncoderDelta { id, delta } => {
                info!("encoder {} Δ{} (via PIC UART)", id, delta);
            }

            // ---- OLED CS handshake — must be forwarded --------------------
            pic::Event::OledSelected => {
                pic::notify_oled_selected();
            }
            pic::Event::OledDeselected => {
                pic::notify_oled_deselected();
            }

            // ---- Misc ------------------------------------------------------
            pic::Event::FirmwareVersion(v) => info!("PIC fw v{}", v),
            pic::Event::NoPresses => {}
        }
    }
}
