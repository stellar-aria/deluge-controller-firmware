use embassy_time::Timer;
use log::debug;

/// Heartbeat blink — P6_7 toggles every 500 ms so we can tell the CPU is alive.
#[embassy_executor::task]
pub(crate) async fn blink_task() {
    debug!("blink_task: started");
    let mut on = false;
    loop {
        on = !on;
        unsafe { rza1l_hal::gpio::write(6, 7, on) };
        Timer::after_millis(500).await;
    }
}
