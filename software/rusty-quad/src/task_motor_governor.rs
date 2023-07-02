use defmt::{info, warn};
use embassy_rp::peripherals::{PIO0};
use embassy_sync::pubsub::DynSubscriber;
use embassy_time::{Duration, Timer, with_timeout};
use quad_dshot_pio::{dshot_embassy_rp::QuadDshotPio, QuadDshotTrait};

use crate::U16x4;

/// Task to govern the arming, disarming and speed settings of the motors.
#[embassy_executor::task]
pub async fn motor_governor(
    mut quad_pio_motors: QuadDshotPio<'static,PIO0>,
    mut set_speeds_ch: DynSubscriber<'static, U16x4>,
    mut arm_motors_ch: DynSubscriber<'static, bool>,
    reverse_motor : (bool,bool,bool,bool),
    timeout: Duration,
) {
    loop {
        // Wait for signal to arm motors
        while !arm_motors_ch.next_message_pure().await {}

        // Send minimum throttle for a few seconds to let
        // the ESCs know that they should be armed
        info!("MOTOR_GOVERNOR : Initializing motors");
        Timer::after(Duration::from_millis(500)).await;
        for _i in 0..50 {
            quad_pio_motors.throttle_minimum();
            Timer::after(Duration::from_millis(50)).await;
        }

        // Set motor directions for the four motors
        info!("MOTOR_GOVERNOR : Setting motor directions");
        for _i in 0..10 {
            quad_pio_motors.reverse(reverse_motor);
            Timer::after(Duration::from_millis(50)).await;
        }

        info!("MOTOR_GOVERNOR : Entering main loop");
        loop {
            // Break if commanded to disarm
            if Some(false) == arm_motors_ch.try_next_message_pure() {
                warn!("MOTOR_GOVERNOR : Disarming motors -> commanded by parser");
                break;
            }

            // Retrieve speeds from channel and transmit, break on timeout
            if let Ok(speeds) = with_timeout(timeout, set_speeds_ch.next_message_pure()).await {
                quad_pio_motors.throttle_clamp(speeds);
            } else {
                warn!("MOTOR_GOVERNOR : Disarming motors -> message timeout");
                break;
            }
        }
    }
}
