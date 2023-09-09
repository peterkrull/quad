use defmt::*;
use embassy_futures::select::{select, Either};

use crate::task_attitude_controller::Attitude;
use crate::signals;

static TASK_ID : &str = "MOTOR_MIXING";

#[embassy_executor::task]
pub async fn motor_mixing(
    mut in_sg_thrust_cmd: signals::ThrustActuateSub,
    mut in_sg_attitude_cmd: signals::AttitudeActuateSub,
    mut in_sg_motor_arm: signals::MotorArmSub,
    out_sg_motor_speed: signals::MotorSpeedPub,
) {

    let mut arm_motors = false;

    let mut attitude = Attitude::new(0., 0., 0.);
    let mut thrust = 0.0;

    info!("{} : Entering main loop",TASK_ID);
    loop {

        match select(in_sg_attitude_cmd.next_message_pure(), in_sg_thrust_cmd.next_message_pure()).await {
            Either::First(a) => attitude = a,
            Either::Second(t) => thrust = t,
        }

        let command = motor_mixing_matrix(thrust, attitude.pitch, attitude.roll, attitude.yaw, 70., 2047.);

        if let Some(arm) = in_sg_motor_arm.try_next_message_pure() {
            arm_motors = arm
        }

        match arm_motors {
            true => out_sg_motor_speed.publish_immediate(Some(command)),
            false => out_sg_motor_speed.publish_immediate(None)
        }

    }
}

/// Basic quad-copter motor mixing function
fn motor_mixing_matrix(thrust: f32, pitch: f32, roll: f32, yaw: f32, min: f32, max: f32) -> (u16,u16,u16,u16) {
    (
        (thrust + pitch + roll + yaw).clamp(min, max) as u16,
        (thrust - pitch - roll + yaw).clamp(min, max) as u16,
        (thrust + pitch - roll - yaw).clamp(min, max) as u16,
        (thrust - pitch + roll - yaw).clamp(min, max) as u16,
    )
}