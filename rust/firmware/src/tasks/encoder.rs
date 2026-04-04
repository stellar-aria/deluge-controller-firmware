use core::sync::atomic::{AtomicI8, Ordering};
use embassy_sync::waitqueue::AtomicWaker;
use log::info;

use crate::events::{EVENT_CHANNEL, HardwareEvent};
use deluge_bsp::{pic, scux_dvu_path};

/// Per-encoder signed delta accumulators, written by IRQ/TINT ISRs.
#[allow(clippy::declare_interior_mutable_const)]
pub(crate) static ENCODER_DELTAS: [AtomicI8; 6] = [
    AtomicI8::new(0),
    AtomicI8::new(0),
    AtomicI8::new(0),
    AtomicI8::new(0),
    AtomicI8::new(0),
    AtomicI8::new(0),
];

/// Wakes `encoder_task` whenever any encoder delta becomes non-zero.
pub(crate) static ENCODER_WAKER: AtomicWaker = AtomicWaker::new();

/// Map a 0–32 level to a 4-segment brightness bar.
///
/// Each 8-step increment fills one more segment.  Partial steps produce
/// intermediate brightness so movement is perceptible at any speed.
fn knob_brightnesses(level: i32) -> [u8; 4] {
    let mut b = [0u8; 4];
    for (i, slot) in b.iter_mut().enumerate() {
        let base = (i as i32) * 8;
        *slot = if level >= base + 8 {
            255
        } else if level > base {
            ((level - base) * 32) as u8 // 1‥7 → 32‥224
        } else {
            0
        };
    }
    b
}

/// Map a gold-knob level (0–32) to a DVU VOLxR value.
///
/// | Level | Volume |
/// |-------|--------|
/// | 0     | 0x0000_0000 (−∞ dB, mute) |
/// | 16    | 0x0010_0000 (0 dB, unity) |
/// | 32    | 0x007F_FFFF (+18 dB, max boost) |
fn dvu_vol_from_knob(level: i32) -> u32 {
    if level <= 0 {
        0
    } else if level <= 16 {
        // Attenuation zone: 0 → 0x0010_0000 linearly.
        (level as u32) * 0x0010_0000 / 16
    } else {
        // Boost zone: 0x0010_0000 → 0x007F_FFFF linearly.
        0x0010_0000 + (level as u32 - 16) * (0x007F_FFFF - 0x0010_0000) / 16
    }
}

/// Called from each encoder's IRQ handler.
///
/// Reads the IRQ pin's new level and the companion pin's current level, derives
/// CW/CCW direction using quadrature decode, and adds ±1 to the per-encoder
/// `ENCODER_DELTAS` accumulator.  Then clears the IRQRR pending flag and wakes
/// `encoder_task`.
///
/// `invert`: set `true` when the IRQ is wired to the electrical *B* signal of
/// the encoder (only TEMPO, where P1.6 is the only pin with an available IRQ
/// function).  This flips the direction formula to match the B-first convention
/// used by the original polling decoder.
fn enc_irq_handler(enc_idx: usize, irq_pin: u8, companion: u8, irq_num: u8, invert: bool) {
    // Read the whole port in one instruction so irq_pin and companion are
    // sampled at the same instant, eliminating the direction-decode race.
    let pins = unsafe { rza1::gpio::read_port(1) };
    let irq_new = (pins >> irq_pin) & 1 != 0;
    let comp = (pins >> companion) & 1 != 0;
    // A-first (IRQ on A): CW when irq_new == comp  → matches old code's A-branch formula.
    // B-first (IRQ on B, invert=true): CW when irq_new != comp.
    let cw = if invert {
        irq_new != comp
    } else {
        irq_new == comp
    };
    ENCODER_DELTAS[enc_idx].fetch_add(if cw { 1 } else { -1 }, Ordering::Relaxed);
    unsafe { rza1::gic::clear_irq_pending(irq_num) };
    ENCODER_WAKER.wake();
}

/// Initialise interrupt-driven quadrature encoder inputs.
///
/// For each encoder, the IRQ-capable pin is muxed to its IRQ0–7 alternate
/// function (alt-func 2) and configured for both-edge detection via ICR1.
/// The companion pin is left as a plain GPIO input for direction sampling.
///
/// # Pin assignment
/// | enc | IRQ pin | IRQ# | GIC ID | companion     |
/// |-----|---------|------|--------|---------------|
/// |  0 SCROLL_X  | P1.11 | IRQ3 | 35 | P1.12 (GPIO) |
/// |  1 TEMPO     | P1.6  | IRQ2 | 34 | P1.7  (GPIO) | ← IRQ on electrical B
/// |  2 MOD_0     | P1.0  | IRQ4 | 36 | P1.15 (GPIO) |
/// |  3 MOD_1     | P1.5  | IRQ1 | 33 | P1.4  (GPIO) |
/// |  4 SCROLL_Y  | P1.8  | IRQ0 | 32 | P1.10 (GPIO) |
/// |  5 SELECT    | P1.2  | IRQ6 | 38 | P1.3  (GPIO) |
///
/// # Safety
/// Must be called before `cortex_ar::interrupt::enable()`.
pub(crate) unsafe fn encoder_irq_init() {
    unsafe {
        // (irq_pin, companion_pin, gic_id, irq_num, invert)
        const SETUP: [(u8, u8, u16, u8, bool); 6] = [
            (11, 12, 35, 3, false), // 0 SCROLL_X
            (6, 7, 34, 2, true),    // 1 TEMPO  (IRQ on electrical B / pin_b)
            (0, 15, 36, 4, false),  // 2 MOD_0
            (5, 4, 33, 1, false),   // 3 MOD_1
            (8, 10, 32, 0, false),  // 4 SCROLL_Y
            (2, 3, 38, 6, false),   // 5 SELECT
        ];

        for &(irq_pin, comp_pin, _, _, _) in &SETUP {
            // Mux IRQ pin to IRQ function (2nd alt); enable PIBC so PPR is readable.
            rza1::gpio::set_pin_mux(1, irq_pin, 2);
            rza1::gpio::enable_input_buffer(1, irq_pin);
            // Companion stays plain GPIO input.
            rza1::gpio::set_as_input(1, comp_pin);
        }

        for &(_, _, _, irq_num, _) in &SETUP {
            rza1::gic::set_irq_both_edges(irq_num);
        }

        rza1::gic::register(35, || enc_irq_handler(0, 11, 12, 3, false));
        rza1::gic::register(34, || enc_irq_handler(1, 6, 7, 2, true));
        rza1::gic::register(36, || enc_irq_handler(2, 0, 15, 4, false));
        rza1::gic::register(33, || enc_irq_handler(3, 5, 4, 1, false));
        rza1::gic::register(32, || enc_irq_handler(4, 8, 10, 0, false));
        rza1::gic::register(38, || enc_irq_handler(5, 2, 3, 6, false));
        for &(_, _, gic_id, _, _) in &SETUP {
            // Priority must be < 31; GICC_PMR is set to 31 so priority=31 is blocked.
            rza1::gic::set_priority(gic_id, 14);
            rza1::gic::enable(gic_id);
        }
        info!("encoder: interrupt-driven init complete (IRQ0/1/2/3/4/6/7 → GIC 32–36, 38–39)");
    }
}

/// Quadrature encoder task — interrupt-driven.
///
/// Sleeps via [`AtomicWaker`] until any encoder IRQ fires, then drains all
/// six [`ENCODER_DELTAS`] accumulators and emits detent events.
///
/// Each encoder's IRQ pin fires on **both edges**, contributing ±1 per
/// transition.  With 2 A-edges per physical detent the threshold of 2 gives a
/// clean 1:1 mapping between physical clicks and emitted detents.
///
/// Gold knobs (encoder ids 2 = LowerGold, 3 = UpperGold) additionally drive
/// their indicator rings via [`pic::set_gold_knob_indicators`].
#[embassy_executor::task]
pub(crate) async fn encoder_task() {
    let mut enc_pos = [0i8; 6];
    // MOD knob levels: 0 = −∞ dB, 16 = 0 dB (unity), 32 = +18 dB.
    // Init at unity so audio is heard at full level on boot.
    let mut knob_level = [16i32; 2];

    info!("encoder_task: started (interrupt-driven)");

    loop {
        // Sleep until an ISR increments at least one delta.
        core::future::poll_fn(|cx| {
            ENCODER_WAKER.register(cx.waker());
            // Check AFTER registering to guarantee no lost wakeup.
            if ENCODER_DELTAS
                .iter()
                .any(|d| d.load(Ordering::Relaxed) != 0)
            {
                core::task::Poll::Ready(())
            } else {
                core::task::Poll::Pending
            }
        })
        .await;

        for i in 0..6usize {
            let delta = ENCODER_DELTAS[i].swap(0, Ordering::Relaxed);
            if delta == 0 {
                continue;
            }

            enc_pos[i] = enc_pos[i].saturating_add(delta);
            let mut detents: i8 = 0;
            while enc_pos[i] > 1 {
                enc_pos[i] -= 2;
                detents += 1;
            }
            while enc_pos[i] < -1 {
                enc_pos[i] += 2;
                detents -= 1;
            }

            if detents != 0 {
                let _ = EVENT_CHANNEL.try_send(HardwareEvent::EncoderRotated {
                    id: i as u8,
                    delta: detents,
                });
                if i == 2 || i == 3 {
                    let k = i - 2;
                    knob_level[k] = (knob_level[k] + detents as i32).clamp(0, 32);
                    let brightness = knob_brightnesses(knob_level[k]);
                    pic::set_gold_knob_indicators(k as u8, brightness).await;
                    info!("knob {} level {} → {:?}", k, knob_level[k], brightness);
                    if k == 0 {
                        // Knob 0 (MOD_0) controls DVU master volume.
                        // 0–16: −∞ dB (0x000_0000) to 0 dB (0x0010_0000) linear
                        // 16–32: 0 dB (0x0010_0000) to +18 dB (0x007F_FFFF) linear
                        let vol = dvu_vol_from_knob(knob_level[0]);
                        unsafe { scux_dvu_path::set_volume_stereo(vol) };
                    }
                } else {
                    info!("encoder {} Δ{}", i, detents);
                }
            }
        }
    }
}
