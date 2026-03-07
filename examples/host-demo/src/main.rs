mod deluge;

use anyhow::Result;
use clap::Parser;
use deluge::{Deluge, DelugeEvent, OLED_FRAMEBUFFER_SIZE, OLED_HEIGHT, OLED_TOPMOST_PIXEL, OLED_VISIBLE_HEIGHT, OLED_WIDTH, PAD_COLS, PAD_ROWS};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Instant;

// The Deluge has two gold (mod) encoders, each with a 4-LED vertical indicator bar.
// Encoder IDs for the gold knobs:
const GOLD_KNOB_0_ENC_ID: u8 = 2; // MOD_0 → knob indicator bar 0
const GOLD_KNOB_1_ENC_ID: u8 = 3; // MOD_1 → knob indicator bar 1

// Knob indicator range: 0 (all off) – 4 (all on).
const KNOB_MAX: i32 = 4;

// Select encoder ID.
const SELECT_ENC_ID: u8 = 5;

// Brightness range: 0 (dimmest) – 25 (brightest).
const BRIGHTNESS_MAX: i32 = 25;

// Tempo encoder button: toggling this only controls the SYNCED_LED GPIO, no PIC LED.
const TEMPO_ENC_BUTTON_ID: u8 = 144 + 4 + 1 * 9; // 157

/// Mutable demo state shared across the main loop.
struct DemoState {
    /// Current indicator level for each of the two gold knob bars (0–KNOB_MAX).
    knob_levels: [i32; 2],
    /// Toggle state of each button LED (index = button LED index 0..=35).
    button_leds: [bool; 36],
    /// Toggle state of the tempo external-synced GPIO LED.
    synced_led: bool,
    /// Current brightness level (0 = dimmest, 25 = brightest).
    brightness: i32,
}

#[derive(Parser)]
#[command(name = "deluge-host-demo", about = "Demo: gradient pads, OLED animation, button readout")]
struct Cli {
    /// Serial port name (auto-detects if omitted)
    #[arg(short, long)]
    port: Option<String>,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    // ── Connect ──────────────────────────────────────────────────────
    let mut deluge = match &cli.port {
        Some(name) => Deluge::open_port(name)?,
        None => Deluge::open()?,
    };

    // First, try to drain any pending data from the device
    println!("Draining any pending data...");
    loop {
        match deluge.try_read_event() {
            Ok(Some(event)) => println!("  (draining: {event:?})"),
            Ok(None) => break,
            Err(e) => {
                println!("  (drain read error, ignoring: {e})");
                break;
            }
        }
    }

    println!("Pinging Deluge...");
    match deluge.ping() {
        Ok(()) => println!("Pong received!"),
        Err(e) => {
            eprintln!("Ping failed: {e:#}");
            eprintln!("Continuing anyway to test event reading...");
        }
    }

    match deluge.get_version() {
        Ok((major, minor, patch)) => println!("Firmware version: {major}.{minor}.{patch}"),
        Err(e) => eprintln!("Could not get version: {e:#}"),
    }

    // Ctrl-C handler
    let running = Arc::new(AtomicBool::new(true));
    {
        let r = running.clone();
        ctrlc::set_handler(move || {
            r.store(false, Ordering::SeqCst);
        })?;
    }

    let start = Instant::now();
    let mut frame: u64 = 0;
    let mut oled_fb = [0u8; OLED_FRAMEBUFFER_SIZE];

    // Initialise interactive state and send initial indicator state to hardware
    let mut state = DemoState {
        knob_levels: [KNOB_MAX / 2; 2],
        button_leds: [false; 36],
        synced_led: false,
        brightness: BRIGHTNESS_MAX,
    };
    apply_knob_indicator(&mut deluge, &state, 0)?;
    apply_knob_indicator(&mut deluge, &state, 1)?;
    deluge.set_brightness(state.brightness as u8)?;

    println!("\nRunning demo (Ctrl-C to exit)...");
    println!("  - Pads: rainbow gradient that scrolls over time");
    println!("  - OLED: bouncing ball + frame counter");
    println!("  - SELECT encoder: adjust brightness (0–25)");
    println!("  - Input events printed to console\n");

    while running.load(Ordering::SeqCst) {
        let t = start.elapsed().as_secs_f64();

        // ── 1. Animated pad gradient (bulk update) ───────────────────
        update_pad_gradient(&mut deluge, t)?;

        // ── 2. OLED animation ────────────────────────────────────────
        // display_update on the firmware side coalesces transfers, so it is
        // safe to call every frame — no SPI queue flooding can occur.
        render_oled_frame(&mut oled_fb, t, frame);
        deluge.update_display(&oled_fb)?;

        // ── 3. Read & print input events ─────────────────────────────
        drain_events(&mut deluge, &mut state)?;

        frame += 1;

        // ~60 fps pacing
        std::thread::sleep(std::time::Duration::from_millis(16));
    }

    // Clean up on exit
    println!("\nCleaning up...");
    deluge.clear_pads()?;
    deluge.clear_display()?;
    deluge.clear_all_leds()?;
    println!("Done.");
    Ok(())
}

// ── Pad gradient ─────────────────────────────────────────────────────
//
// HSV rainbow that shifts hue by column and scrolls over time.

fn update_pad_gradient(deluge: &mut Deluge, t: f64) -> Result<()> {
    let mut colors = [[0u8; 3]; PAD_COLS as usize * PAD_ROWS as usize]; // col-major: index = col * 8 + row
    for col in 0..PAD_COLS {
        for row in 0..PAD_ROWS {
            let hue = ((col as f64 / PAD_COLS as f64) + t * 0.15) % 1.0;
            let value = 0.3 + 0.7 * (row as f64 / (PAD_ROWS - 1) as f64);
            let (r, g, b) = hsv_to_rgb(hue, 1.0, value);
            colors[(col as usize) * PAD_ROWS as usize + (row as usize)] = [r, g, b];
        }
    }
    deluge.set_all_pads(&colors)
}

fn hsv_to_rgb(h: f64, s: f64, v: f64) -> (u8, u8, u8) {
    let h = (h % 1.0) * 6.0;
    let i = h.floor() as u32;
    let f = h - i as f64;
    let p = v * (1.0 - s);
    let q = v * (1.0 - s * f);
    let t = v * (1.0 - s * (1.0 - f));
    let (r, g, b) = match i % 6 {
        0 => (v, t, p),
        1 => (q, v, p),
        2 => (p, v, t),
        3 => (p, q, v),
        4 => (t, p, v),
        _ => (v, p, q),
    };
    (
        (r * 255.0) as u8,
        (g * 255.0) as u8,
        (b * 255.0) as u8,
    )
}

// ── OLED animation ───────────────────────────────────────────────────
//
// A bouncing ball with a text-like frame counter rendered as a simple
// bitmap font. The framebuffer uses vertical page encoding:
//   fb[page * 128 + x] has 8 vertical pixels, LSB = top of page.

fn render_oled_frame(fb: &mut [u8; OLED_FRAMEBUFFER_SIZE], t: f64, frame: u64) {
    // Clear
    fb.fill(0);

    // NOTE: the top OLED_TOPMOST_PIXEL (5) rows of the framebuffer are hidden by the
    // bezel — the visible area is rows OLED_TOPMOST_PIXEL..OLED_HEIGHT (0..43 visible).
    // All drawing is offset by OLED_TOPMOST_PIXEL so content lands in the visible region.
    let top = OLED_TOPMOST_PIXEL as i32;
    let vis_h = OLED_VISIBLE_HEIGHT as i32;

    // ── Bouncing ball ────────────────────────────────────────────────
    let ball_r = 6i32;
    let bx = ((t * 45.0).sin() * 0.5 + 0.5) * (OLED_WIDTH as f64 - 2.0 * ball_r as f64) + ball_r as f64;
    let by = top as f64
        + ((t * 60.0).sin() * 0.5 + 0.5) * (vis_h as f64 - 2.0 * ball_r as f64)
        + ball_r as f64;
    draw_filled_circle(fb, bx as i32, by as i32, ball_r);

    // ── Sine wave across the middle ──────────────────────────────────
    for x in 0..OLED_WIDTH as i32 {
        let y = top + (vis_h / 2)
            + ((x as f64 * 0.08 + t * 3.0).sin() * 10.0) as i32;
        if y >= top && y < OLED_HEIGHT as i32 {
            set_pixel(fb, x, y);
        }
    }

    // ── Frame counter in top-left of visible area ────────────────────
    let text = format!("F:{frame}");
    draw_string(fb, 1, top + 1, &text);
}

/// Set a single pixel in the page-encoded framebuffer.
fn set_pixel(fb: &mut [u8; OLED_FRAMEBUFFER_SIZE], x: i32, y: i32) {
    if x < 0 || x >= OLED_WIDTH as i32 || y < 0 || y >= OLED_HEIGHT as i32 {
        return;
    }
    let page = y as usize / 8;
    let bit = y as usize % 8;
    fb[page * OLED_WIDTH + x as usize] |= 1 << bit;
}

fn draw_filled_circle(fb: &mut [u8; OLED_FRAMEBUFFER_SIZE], cx: i32, cy: i32, r: i32) {
    for dy in -r..=r {
        for dx in -r..=r {
            if dx * dx + dy * dy <= r * r {
                set_pixel(fb, cx + dx, cy + dy);
            }
        }
    }
}

// ── Tiny 5x7 bitmap font (digits, colon, uppercase letters) ─────────

/// Each character is 5 columns wide. Each column is 7 bits (bit 0 = top).
const FONT_5X7: &[(char, [u8; 5])] = &[
    ('0', [0x3E, 0x51, 0x49, 0x45, 0x3E]),
    ('1', [0x00, 0x42, 0x7F, 0x40, 0x00]),
    ('2', [0x42, 0x61, 0x51, 0x49, 0x46]),
    ('3', [0x21, 0x41, 0x45, 0x4B, 0x31]),
    ('4', [0x18, 0x14, 0x12, 0x7F, 0x10]),
    ('5', [0x27, 0x45, 0x45, 0x45, 0x39]),
    ('6', [0x3C, 0x4A, 0x49, 0x49, 0x30]),
    ('7', [0x01, 0x71, 0x09, 0x05, 0x03]),
    ('8', [0x36, 0x49, 0x49, 0x49, 0x36]),
    ('9', [0x06, 0x49, 0x49, 0x29, 0x1E]),
    (':', [0x00, 0x36, 0x36, 0x00, 0x00]),
    ('F', [0x7F, 0x09, 0x09, 0x09, 0x01]),
    ('P', [0x7F, 0x09, 0x09, 0x09, 0x06]),
    ('R', [0x7F, 0x09, 0x19, 0x29, 0x46]),
    ('E', [0x7F, 0x49, 0x49, 0x49, 0x41]),
    ('S', [0x46, 0x49, 0x49, 0x49, 0x31]),
    ('D', [0x7F, 0x41, 0x41, 0x22, 0x1C]),
    ('B', [0x7F, 0x49, 0x49, 0x49, 0x36]),
    ('A', [0x7E, 0x11, 0x11, 0x11, 0x7E]),
    ('L', [0x7F, 0x40, 0x40, 0x40, 0x40]),
    ('O', [0x3E, 0x41, 0x41, 0x41, 0x3E]),
    ('N', [0x7F, 0x04, 0x08, 0x10, 0x7F]),
    ('T', [0x01, 0x01, 0x7F, 0x01, 0x01]),
    (' ', [0x00, 0x00, 0x00, 0x00, 0x00]),
    ('I', [0x00, 0x41, 0x7F, 0x41, 0x00]),
    ('C', [0x3E, 0x41, 0x41, 0x41, 0x22]),
    ('G', [0x3E, 0x41, 0x49, 0x49, 0x3A]),
    ('H', [0x7F, 0x08, 0x08, 0x08, 0x7F]),
    ('K', [0x7F, 0x08, 0x14, 0x22, 0x41]),
    ('M', [0x7F, 0x02, 0x0C, 0x02, 0x7F]),
    ('U', [0x3F, 0x40, 0x40, 0x40, 0x3F]),
    ('V', [0x1F, 0x20, 0x40, 0x20, 0x1F]),
    ('W', [0x3F, 0x40, 0x38, 0x40, 0x3F]),
    ('X', [0x63, 0x14, 0x08, 0x14, 0x63]),
    ('Y', [0x07, 0x08, 0x70, 0x08, 0x07]),
    ('Z', [0x61, 0x51, 0x49, 0x45, 0x43]),
];

fn draw_char(fb: &mut [u8; OLED_FRAMEBUFFER_SIZE], x0: i32, y0: i32, ch: char) {
    let upper = ch.to_ascii_uppercase();
    let glyph = FONT_5X7
        .iter()
        .find(|(c, _)| *c == upper)
        .map(|(_, g)| g);
    let glyph = match glyph {
        Some(g) => g,
        None => return, // unsupported char, skip
    };
    for (dx, col_bits) in glyph.iter().enumerate() {
        for bit in 0..7 {
            if col_bits & (1 << bit) != 0 {
                set_pixel(fb, x0 + dx as i32, y0 + bit);
            }
        }
    }
}

fn draw_string(fb: &mut [u8; OLED_FRAMEBUFFER_SIZE], x0: i32, y0: i32, s: &str) {
    let mut x = x0;
    for ch in s.chars() {
        draw_char(fb, x, y0, ch);
        x += 6; // 5 pixels + 1 gap
    }
}

// ── Event drain ──────────────────────────────────────────────────────

fn drain_events(deluge: &mut Deluge, state: &mut DemoState) -> Result<()> {
    loop {
        match deluge.try_read_event()? {
            Some(event) => {
                print_event(&event);
                handle_event(deluge, state, &event)?;
            }
            None => break,
        }
    }
    Ok(())
}

/// Apply interactive LED changes triggered by an event.
fn handle_event(deluge: &mut Deluge, state: &mut DemoState, event: &DelugeEvent) -> Result<()> {
    match event {
        // Gold encoder turns: adjust the corresponding 4-LED indicator bar.
        // SELECT encoder turn: adjust brightness.
        DelugeEvent::EncoderRotated { id, delta } => {
            match *id {
                x if x == GOLD_KNOB_0_ENC_ID || x == GOLD_KNOB_1_ENC_ID => {
                    let knob = if *id == GOLD_KNOB_0_ENC_ID { 0usize } else { 1usize };
                    state.knob_levels[knob] =
                        (state.knob_levels[knob] + *delta as i32).clamp(0, KNOB_MAX);
                    apply_knob_indicator(deluge, state, knob)?;
                }
                x if x == SELECT_ENC_ID => {
                    state.brightness =
                        (state.brightness + *delta as i32).clamp(0, BRIGHTNESS_MAX);
                    deluge.set_brightness(state.brightness as u8)?;
                    println!("  [brightness] {}", state.brightness);
                }
                _ => {}
            }
        }
        // Tempo encoder button: only toggles the GPIO synced LED, no PIC LED.
        DelugeEvent::ButtonPressed { id } if *id == TEMPO_ENC_BUTTON_ID => {
            state.synced_led = !state.synced_led;
            deluge.set_synced_led(state.synced_led)?;
        }
        // Button press: toggle the corresponding indicator LED.
        // Button id is the raw PIC value (144–179); LED index = id - 144.
        // Gold knob presses (modEncoder0 = 162, modEncoder1 = 171) are excluded —
        // their indicator bars are controlled by rotation only.
        DelugeEvent::ButtonPressed { id } if *id >= 144 && *id < 180
            && *id != 162  // modEncoder0ButtonCoord {0,2} → 144+18
            && *id != 171  // modEncoder1ButtonCoord {0,3} → 144+27
        => {
            let led = *id - 144;
            let idx = led as usize;
            state.button_leds[idx] = !state.button_leds[idx];
            deluge.set_led(led, state.button_leds[idx])?;
        }
        _ => {}
    }
    Ok(())
}

/// Set the 4-LED gold knob indicator bar for `knob` (0 or 1) from the current level.
/// LEDs light up from the bottom; `level` LEDs are on.
fn apply_knob_indicator(deluge: &mut Deluge, state: &DemoState, knob: usize) -> Result<()> {
    let level = state.knob_levels[knob];
    let levels = [
        if level > 0 { 255u8 } else { 0 },
        if level > 1 { 255u8 } else { 0 },
        if level > 2 { 255u8 } else { 0 },
        if level > 3 { 255u8 } else { 0 },
    ];
    deluge.set_knob_indicator(knob as u8, &levels)
}

fn print_event(event: &DelugeEvent) {
    match event {
        DelugeEvent::PadPressed { col, row } => {
            println!("[PAD]     pressed  col={col} row={row}");
        }
        DelugeEvent::PadReleased { col, row } => {
            println!("[PAD]     released col={col} row={row}");
        }
        DelugeEvent::ButtonPressed { id } => {
            println!("[BUTTON]  pressed  id={id}");
        }
        DelugeEvent::ButtonReleased { id } => {
            println!("[BUTTON]  released id={id}");
        }
        DelugeEvent::EncoderRotated { id, delta } => {
            println!("[ENCODER] rotated  id={id} delta={delta}");
        }
        DelugeEvent::EncoderPressed { id } => {
            println!("[ENCODER] pressed  id={id}");
        }
        DelugeEvent::EncoderReleased { id } => {
            println!("[ENCODER] released id={id}");
        }
        DelugeEvent::Version {
            major,
            minor,
            patch,
        } => {
            println!("[VERSION] {major}.{minor}.{patch}");
        }
        DelugeEvent::Pong => {
            println!("[PONG]");
        }
        DelugeEvent::Ready => {
            println!("[READY]");
        }
        DelugeEvent::Error(msg) => {
            println!("[ERROR]   {msg}");
        }
    }
}
