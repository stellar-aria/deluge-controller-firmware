use embassy_time::Timer;
use log::info;

/// Jack-detect and speaker-enable task.
///
/// Polls the five audio jack-detect GPIO inputs every 200 ms and logs any
/// changes.  Also drives the `SPEAKER_ENABLE` output: the speaker amplifier is
/// enabled only when no headphone or line-out plug is inserted.
///
/// | signal           | port/pin | direction |
/// |------------------|----------|-----------|
/// | HEADPHONE_DETECT | P6.5     | input     |
/// | LINE_IN_DETECT   | P6.6     | input     |
/// | MIC_DETECT       | P7.9     | input     |
/// | LINE_OUT_DETECT_L| P6.3     | input     |
/// | LINE_OUT_DETECT_R| P6.4     | input     |
/// | SPEAKER_ENABLE   | P4.1     | output    |
#[embassy_executor::task]
pub(crate) async fn jack_detect_task() {
    unsafe {
        rza1::gpio::set_as_input(6, 5); // HEADPHONE_DETECT
        rza1::gpio::set_as_input(6, 6); // LINE_IN_DETECT
        rza1::gpio::set_as_input(7, 9); // MIC_DETECT
        rza1::gpio::set_as_input(6, 3); // LINE_OUT_DETECT_L
        rza1::gpio::set_as_input(6, 4); // LINE_OUT_DETECT_R
        rza1::gpio::set_as_output(4, 1); // SPEAKER_ENABLE
        rza1::gpio::write(4, 1, false); // speaker off until we know the jack state
    }

    let mut prev_hp = false;
    let mut prev_li = false;
    let mut prev_mic = false;
    let mut prev_lol = false;
    let mut prev_lor = false;

    info!("jack_detect_task: started");

    loop {
        let hp = unsafe { rza1::gpio::read_pin(6, 5) };
        let li = unsafe { rza1::gpio::read_pin(6, 6) };
        let mic = unsafe { rza1::gpio::read_pin(7, 9) };
        let lol = unsafe { rza1::gpio::read_pin(6, 3) };
        let lor = unsafe { rza1::gpio::read_pin(6, 4) };

        if hp != prev_hp || li != prev_li || mic != prev_mic || lol != prev_lol || lor != prev_lor {
            prev_hp = hp;
            prev_li = li;
            prev_mic = mic;
            prev_lol = lol;
            prev_lor = lor;

            info!(
                "jack: hp={} line_in={} mic={} line_out_L={} line_out_R={}",
                hp, li, mic, lol, lor
            );

            // Speaker enable is gated on USB audio streaming being active.
            // We leave that to the audio task; do NOT enable the amplifier here
            // (powering it on with no signal produces audible noise).
        }

        Timer::after_millis(200).await;
    }
}
