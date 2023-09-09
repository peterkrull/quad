use core::f32::consts::PI;

use defmt::*;

use crate::{
    task_attitude_controller::{Attitude, StabilizationMode::{Horizon,Acro, self}},
    functions::{mapf, self},
    sbus_cmd::TriSwitch,
    task_blinker::BlinkerMode,
    signals
};

static TASK_ID : &str = "COMMANDER";

#[embassy_executor::task]
pub async fn commander(
    mut in_sbus_cmd: signals::SbusCmdSub,
    mut in_motor_state : signals::MotorStateSub,
    mut in_attitude_sense : signals::AttitudeSenseSub,
    mut in_flight_mode: signals::FlightModeSub,
    out_motor_arm: signals::MotorArmPub,
    out_motor_dir: signals::MotorDirPub,
    out_thrust_cmd: signals::ThrustActuatePub,
    out_blinker_mode: signals::BlinkerModePub,
    out_attitude_int_reset : signals::AttitudeIntResetPub,
    out_attitude_stab_mode : signals::AttitudeStabModePub,
) {

    out_motor_dir.publish_immediate((true,true,false,false));
    out_blinker_mode.publish_immediate(BlinkerMode::OnOffFast);

    let mut ref_yaw = 0.0;
    let mut current_stab_mode: Option<StabilizationMode> = None;
    let mut current_motor_state = None;
    let mut current_flight_mode = None;

    info!("{} : Entering main loop",TASK_ID);
    loop {

        let mut reset_controllers = false;

        match in_sbus_cmd.next_message_pure().await {

            // Valid command received, send arming switch
            Ok(cmd) => {

                out_motor_arm.publish_immediate(!cmd.sw_b.is_idle());
                
                out_blinker_mode.publish_immediate(match cmd.sw_b {
                    TriSwitch::Idle   => BlinkerMode::OnOffFast,
                    TriSwitch::Middle => BlinkerMode::TwoFast,
                    TriSwitch::Active => BlinkerMode::TwoFast,
                });

                let thrust = thrust_curve(match cmd.sw_c {
                    TriSwitch::Idle   => mapf(cmd.thrust, 0., 1., 400., 1000.),
                    TriSwitch::Middle => mapf(cmd.thrust, 0., 1., 250., 1500.),
                    TriSwitch::Active => mapf(cmd.thrust, 0., 1., 250., 2047.),
                });

                out_thrust_cmd.publish_immediate( thrust );

                let stab_mode = match cmd.sw_f {
                    TriSwitch::Idle | TriSwitch::Middle => {
                        ref_yaw = functions::circular_constraint(ref_yaw - cmd.yaw*0.01, -PI, PI);
                        Horizon(Attitude::new(
                            pitch_curve(cmd.pitch),
                            roll_curve(cmd.roll),
                            yaw_curve(ref_yaw)
                        ))
                    },
                    TriSwitch::Active => {
                        Acro(Attitude::new(
                            pitch_curve(cmd.pitch*10.),
                            roll_curve(cmd.roll*10.),
                            yaw_curve(-cmd.yaw*5.)
                        ))
                    },
                };

                out_attitude_stab_mode.publish_immediate(stab_mode);

                // Check if stabilization mode has changed or is uninitialized
                if current_stab_mode.is_some_and(|c| !c.same_variant_as(&stab_mode)) || current_stab_mode.is_none() {
                    let mode_str = match stab_mode { Horizon(_) => "Horizon", Acro(_) => "Acro" };
                    info!("{} : Stabilization mode changed -> {}",TASK_ID,mode_str);
                    current_stab_mode = Some(stab_mode);
                    reset_controllers = true;
                }

                // Reset controller if craft just changed flight mode
                if let Some(flight_mode) = in_flight_mode.try_next_message_pure() {
                    if current_flight_mode != Some(flight_mode) {
                        current_flight_mode = Some(flight_mode);
                        reset_controllers = true;
                    }
                }                

                // Reset controller integrals if motors was just armed
                if let Some(motor_state) = in_motor_state.try_next_message_pure() {
                    if current_motor_state != Some(motor_state) {
                        current_motor_state = Some(motor_state);
                        reset_controllers = true;
                    }
                }
            },

            // SBUS failsafe, connection to remote control lost
            Err(error) => {
                out_motor_arm.publish_immediate(false);
                reset_controllers = true;
                error!("{} : SBUS error -> {}",TASK_ID,error)
            },
        }

        // Reset integral and yaw angle 
        if reset_controllers {
            out_attitude_int_reset.publish_immediate(true);
            ref_yaw = in_attitude_sense.next_message_pure().await.0.yaw;
        }
    }
}


fn pitch_curve(input : f32) -> f32 {
    input*0.5
}

fn roll_curve(input : f32) -> f32 {
    input*0.5
}

fn yaw_curve(input : f32) -> f32 {
    input
}

fn thrust_curve(input : f32) -> f32 {
    input
}