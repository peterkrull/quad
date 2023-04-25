use embassy_rp::gpio::{Output, AnyPin, Level};
use embassy_time::{Duration, Timer};

#[embassy_executor::task]
async fn blinker(pin : AnyPin) {

    let mut led = Output::new(pin,Level::Low);
    loop {

        led.set_high(); // Short high
        Timer::after(Duration::from_millis(100)).await;

        led.set_low(); // Medium low
        Timer::after(Duration::from_millis(200)).await;

        led.set_high(); // Short high
        Timer::after(Duration::from_millis(100)).await;

        led.set_low(); // Long low
        Timer::after(Duration::from_millis(800)).await;

    }
}