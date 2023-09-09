
use core::{f32::consts::PI, ops::Sub};
use pid_controller_rs::Pid;
use crate::signals;

use defmt::*;

#[derive(Clone,Copy,Format,PartialEq)]
pub struct Attitude {
    pub pitch : f32,
    pub roll : f32,
    pub yaw : f32
}

impl Attitude {
    pub fn new(pitch : f32, roll: f32, yaw: f32) -> Self {
        Attitude {pitch,roll,yaw}
    }
}

impl Sub for Attitude {
    type Output = Attitude;

    fn sub(self, rhs: Self) -> Self::Output {
        Attitude {
            pitch: self.pitch - rhs.pitch,
            roll: self.roll - rhs.roll,
            yaw: self.yaw - rhs.yaw
        }
    }
}

#[derive(Clone,Copy,Format,PartialEq)]
pub enum StabilizationMode {
    Horizon(Attitude),
    Acro(Attitude)
}

impl StabilizationMode {
    pub fn same_variant_as(&self, rhs: &StabilizationMode) -> bool {
        self.variant() == rhs.variant()
    }

    fn variant (&self) -> usize {
        match self {
            StabilizationMode::Horizon(_) => 0,
            StabilizationMode::Acro(_) => 1,
        }
    }
}

static TASK_ID : &str = "ATTITUDE_CONTROLLER";

#[embassy_executor::task]
pub async fn attitude_controller(
    mut in_attitude_sense: signals::AttitudeSenseSub,
    mut in_attitude_int_reset : signals::AttitudeIntResetSub,
    mut in_attitude_stab_mode : signals::AttitudeStabModeSub,
    out_attitude_actuate: signals::AttitudeActuatePub,
    cfg_sample_time : embassy_time::Duration,
) {

    // Convert sample time Duration type to floating point seconds representation
    let sample_time_secs = cfg_sample_time.as_micros() as f32 / 1e6;

    // Aquire satbilization mode
    let mut stabilization_mode = in_attitude_stab_mode.next_message_pure().await;

    // Setup controllers for pitch, roll and yaw, using a cascaded controller scheme.
    let mut pid_pitch_outer = Pid::new( 10., 0.1, 0., true, sample_time_secs );
    let mut pid_pitch_inner = Pid::new( 40., 1.0, 0.01, true, sample_time_secs ).set_lp_filter(0.01);
    let mut pid_roll_outer = Pid::new( 10., 0.1, 0., true, sample_time_secs );
    let mut pid_roll_inner = Pid::new( 30., 1.0, 0.01, true, sample_time_secs ).set_lp_filter(0.01);
    let mut pid_yaw_outer = Pid::new( 8., 0.001, 0., true, sample_time_secs ).set_circular(-PI, PI);
    let mut pid_yaw_inner = Pid::new( 60., 1.0, 0., true, sample_time_secs ).set_circular(-PI, PI).set_lp_filter(0.01);

    info!("{} : Entering main loop",TASK_ID);
    loop {

        // Change stabilization mode if signaled to do so
        crate::signals::try_assign_from_channel(&mut in_attitude_stab_mode, &mut stabilization_mode );

        // Reset integrators if signaled to do so
        if let Some(true) = in_attitude_int_reset.try_next_message_pure() {
            pid_pitch_outer.reset_integral();   pid_pitch_inner.reset_integral();
            pid_roll_outer.reset_integral();    pid_roll_inner.reset_integral();
            pid_yaw_outer.reset_integral();     pid_yaw_inner.reset_integral();
        }

        // Wait for new measurements to arrive
        let (att_angle,att_rate) = in_attitude_sense.next_message_pure().await;

        // Generate actuation signal
        out_attitude_actuate.publish_immediate( match stabilization_mode {

            StabilizationMode::Horizon(reference) => {

                // Run outer part of cascaded control loop
                let outer_error = reference - att_angle;
                let inner_reference = Attitude::new(
                    pid_pitch_outer.update( outer_error.pitch ),
                    pid_roll_outer.update( outer_error.roll ),
                    pid_yaw_outer.update( outer_error.yaw )
                );

                // Run inner part of cascaded control loop
                let inner_error = inner_reference - att_rate;
                Attitude::new(
                    pid_pitch_inner.update( inner_error.pitch ),
                    pid_roll_inner.update( inner_error.roll ),
                    pid_yaw_inner.update( inner_error.yaw )
                )
            }

            StabilizationMode::Acro(reference) => {

                let error = reference - att_rate;
                Attitude::new(
                    pid_pitch_inner.update( error.pitch ),
                    pid_roll_inner.update( error.roll ),
                    pid_yaw_inner.update( error.yaw )
                )
            }
        });
    }
}


