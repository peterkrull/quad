use embassy_rp::gpio::{Output, AnyPin, Level};
use embassy_time::{Duration, Timer};
use crate::signals;

#[embassy_executor::task]
pub async fn blinker(
    mut in_blinker_mode : signals::BlinkerModeSub,
    pin : AnyPin
) {

    let mut blinker_mode = BlinkerMode::None;

    let mut led = Output::new(pin,Level::Low);
    loop { 

        // Try to read new blinker mode
        if let Some(b) = in_blinker_mode.try_next_message_pure() {blinker_mode = b}

        match blinker_mode {
            BlinkerMode::None => {
                led.set_low();
                blinker_mode = in_blinker_mode.next_message_pure().await;
            },
            BlinkerMode::OneFast => one_fast(&mut led).await,
            BlinkerMode::TwoFast => two_fast(&mut led).await,
            BlinkerMode::ThreeFast => three_fast(&mut led).await,
            BlinkerMode::OnOffFast => on_off_fast(&mut led).await,
            BlinkerMode::OnOffSlow => on_off_slow(&mut led).await,
        };
    }
}

#[allow(unused)]
async fn one_fast<'a>(led: &mut Output<'a,AnyPin>) {

    led.set_high(); // Short high
    Timer::after(Duration::from_millis(50)).await;

    led.set_low(); // Long low
    Timer::after(Duration::from_millis(950)).await;
}

#[allow(unused)]
async fn two_fast<'a>(led: &mut Output<'a,AnyPin>) {
    led.set_high(); // Short high
    Timer::after(Duration::from_millis(50)).await;

    led.set_low(); // Medium low
    Timer::after(Duration::from_millis(100)).await;

    led.set_high(); // Short high
    Timer::after(Duration::from_millis(50)).await;

    led.set_low(); // Long low
    Timer::after(Duration::from_millis(800)).await;
}

#[allow(unused)]
async fn three_fast<'a>(led: &mut Output<'a,AnyPin>) {
    led.set_high(); // Short high
    Timer::after(Duration::from_millis(50)).await;

    led.set_low(); // Medium low
    Timer::after(Duration::from_millis(100)).await;

    led.set_high(); // Short high
    Timer::after(Duration::from_millis(50)).await;

    led.set_low(); // Medium low
    Timer::after(Duration::from_millis(100)).await;

    led.set_high(); // Short high
    Timer::after(Duration::from_millis(50)).await;

    led.set_low(); // Long low
    Timer::after(Duration::from_millis(650)).await;
}

#[allow(unused)]
async fn on_off_fast<'a>(led: &mut Output<'a,AnyPin>) {
    led.set_high(); // Short high
    Timer::after(Duration::from_millis(100)).await;

    led.set_low(); // Medium low
    Timer::after(Duration::from_millis(100)).await;
}

#[allow(unused)]
async fn on_off_slow<'a>(led: &mut Output<'a,AnyPin>) {
    led.set_high(); // Short high
    Timer::after(Duration::from_millis(100)).await;

    led.set_low(); // Medium low
    Timer::after(Duration::from_millis(100)).await;
}

#[allow(unused)]
#[derive(Clone,Copy)]
pub enum BlinkerMode {
    None,
    OneFast,
    TwoFast,
    ThreeFast,
    OnOffFast,
    OnOffSlow,
}