
use core::f32::consts::PI;
use num_traits::Float;

use defmt::*;
use embassy_sync::{pubsub::{Publisher, DynSubscriber}, blocking_mutex::raw::CriticalSectionRawMutex};
use embassy_time::Duration;

use ahrs::{Madgwick,Ahrs};
use icm20948_async::Data6Dof;
use pid_controller_rs::Pid;

use crate::{sbus_cmd::{SbusCmd, TwiSwitch, self}, functions::mapf};

type U16x4 = (u16, u16, u16, u16);
type SbusDur = (Option<SbusCmd>, Duration);
type CSMutex = CriticalSectionRawMutex;

#[embassy_executor::task]
pub async fn control_loop(
    motors: Publisher<'static, CSMutex, U16x4, 1, 4, 1>,
    mut motors_init: Publisher<'static, CSMutex, bool, 1, 4, 1>,
    mut imu_data: DynSubscriber<'static, Data6Dof>,
    mut sbus_sub: DynSubscriber<'static, SbusDur>,
    sample_time : Duration,
) {

    // Convert sample time Duration type to floating point seconds representation
    let sample_time_secs = sample_time.as_micros() as f32 / 1e6;

    // Initialize SbusCmd to safe defaults
    let mut user_command = SbusCmd::default();

    // Filter to obtain quaternion from acc, gyr and mag
    let mut ahrs = Madgwick::new(sample_time_secs, 0.01, 0.001);


    // Integrate yaw command to obtain yaw reference (circular PID with I-gain of 1)
    let mut yaw_integrator = Pid::new( 0.0, 1.0, 0.0, false, super::IMU_SAMPLE_TIME_SECS ).set_circular(-PI, PI);

    // Setup controllers for pitch, roll and yaw, using a cascaded controller scheme.
    let mut pid_pitch_outer = Pid::new( 10., 0.1, 0., true, super::IMU_SAMPLE_TIME_SECS );
    let mut pid_pitch_inner = Pid::new( 40., 1.0, 0.01, true, super::IMU_SAMPLE_TIME_SECS ).set_lp_filter(0.02);
    let mut pid_roll_outer = Pid::new( 10., 0.1, 0., true, super::IMU_SAMPLE_TIME_SECS );
    let mut pid_roll_inner = Pid::new( 30., 1.0, 0.01, true, super::IMU_SAMPLE_TIME_SECS ).set_lp_filter(0.02);
    let mut pid_yaw_outer = Pid::new( 8., 0.001, 0., true, super::IMU_SAMPLE_TIME_SECS ).set_circular(-PI, PI);
    let mut pid_yaw_inner = Pid::new( 60., 1.0, 0., true, super::IMU_SAMPLE_TIME_SECS );

    info!("CONTROL_LOOP : Entering main loop");
    loop {
        // Received imu messages
        let data = imu_data.next_message_pure().await;

        if let Ok(q) = ahrs.update_imu(&data.gyr, &data.acc) {
            let (roll, pitch, yaw) = q.euler_angles();

            // Check that sbus command is sane
            sbus_sanity(&mut user_command, &mut motors_init, &mut sbus_sub);

            // Reset integrators if previously non-armed
            if !user_command.sw_b.is_active(){
                pid_pitch_outer.reset_integral();   pid_pitch_inner.reset_integral();
                pid_roll_outer.reset_integral();    pid_roll_inner.reset_integral();
                pid_yaw_outer.reset_integral();     pid_yaw_inner.reset_integral();
                yaw_integrator.reset_integral_to(yaw);
            }

            // Controller selection
            let (pitch_cmd,roll_cmd,yaw_cmd) = match user_command.sw_f {

                // Horizon mode
                TwiSwitch::Idle | sbus_cmd::TwiSwitch::Middle  => {

                    let pitch_outer = pid_pitch_outer.update( user_command.pitch - pitch );
                    let pitch = pid_pitch_inner.update( pitch_outer - data.gyr[1] );

                    let roll_outer = pid_roll_outer.update( user_command.roll - roll );
                    let roll = pid_roll_inner.update( roll_outer - data.gyr[0] );

                    let yaw_int = yaw_integrator.update( -user_command.yaw );
                    let yaw_outer = pid_yaw_outer.update( yaw_int - yaw );
                    let yaw = pid_yaw_inner.update( yaw_outer - data.gyr[2] );

                    (pitch,roll,yaw)
                }

                // Acro mode
                TwiSwitch::Active => {

                    // Define controller gains (temporary)
                    struct Gains { pitch : f32, roll : f32, yaw : f32 }
                    let gains = Gains { pitch : 10.0f32, roll : 10.0f32, yaw : 10.0f32 };

                    let pitch = pid_pitch_inner.update( user_command.pitch*gains.pitch - data.gyr[1] );
                    let roll = pid_roll_inner.update( user_command.roll*gains.roll - data.gyr[0] );
                    let yaw = pid_yaw_inner.update( -user_command.yaw*gains.yaw - data.gyr[2] );

                    (pitch,roll,yaw)
                }
            };

            // Set throttle behavior based on switch C
            let (thrust,motor_max) = match user_command.sw_c {
                TwiSwitch::Idle => (mapf(user_command.thrust, 0., 1., 80., 1000.),1500.),
                TwiSwitch::Middle => (mapf(user_command.thrust, 0., 1., 250., 1000.)/(pitch.cos()*roll.cos()).clamp(0.5, 1.0),2047.),
                TwiSwitch::Active => (mapf(user_command.thrust, 0., 1., 250., 2047.),2047.),
            };

            let command = motor_mixing(thrust, pitch_cmd, roll_cmd, yaw_cmd, 70., motor_max);

            motors.publish_immediate(command);
        }
    }
}

/// Basic quad-copter motor mixing function
fn motor_mixing(thrust: f32, pitch: f32, roll: f32, yaw: f32, min: f32, max: f32) -> U16x4 {
    (
        (thrust + pitch + roll + yaw).clamp(min, max) as u16,
        (thrust - pitch - roll + yaw).clamp(min, max) as u16,
        (thrust + pitch - roll - yaw).clamp(min, max) as u16,
        (thrust - pitch + roll - yaw).clamp(min, max) as u16,
    )
}

fn sbus_sanity(
    sbus_msg_out: &mut SbusCmd,
    ch_motor_init: &mut Publisher<'static, CSMutex, bool, 1, 4, 1>,
    ch_sbus_cmd: &mut DynSubscriber<'static, SbusDur>,
) {
    // If a new message is in the channel
    if let Some((opt_msg, parse_duration)) = ch_sbus_cmd.try_next_message_pure() {
        // Disarm if time since last parse is high (Receiver disconnect)
        if parse_duration > Duration::from_millis(500) {
            ch_motor_init.publish_immediate(false);
        }

        // If a command exists, publish
        if let Some(cmd) = opt_msg {
            let arm_sw = cmd.sw_b.is_active();
            ch_motor_init.publish_immediate(arm_sw);
            *sbus_msg_out = cmd;
        } else {
            ch_motor_init.publish_immediate(false);
        }
    }
}
