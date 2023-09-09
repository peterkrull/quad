use ahrs::{Madgwick,Ahrs};
use defmt::*;
use nalgebra::{Quaternion, Unit, Vector3};

use crate::task_attitude_controller::Attitude;
use crate::signals;

static TASK_ID : &str = "STATE_ESTIMATOR";

#[embassy_executor::task]
pub async fn state_estimator(
    mut in_imu_reading: signals::ImuReadingSub,
    out_attitude_sense: signals::AttitudeSensePub,
    cfg_imu_sample_time : embassy_time::Duration
) {

    let exp_accel = Vector3::from([0.,0.,0.]);

    let sample_time_secs = cfg_imu_sample_time.as_micros() as f32 / 1e6;
    let mut ahrs = Madgwick::new(sample_time_secs, 0.01, 0.001);

    let mut prev_quaternion : Option<Unit<Quaternion<f32>>> = None;

    info!("{} : Entering main loop",TASK_ID);
    loop {
        let imu_data = in_imu_reading.next_message_pure().await;

        let acceleration = prev_quaternion.map_or_else(
            ||imu_data.acc,
            |q|{imu_data.acc - q.to_rotation_matrix()*exp_accel}
        );

        #[cfg(feature = "mag")]
        let ahrs_est = ahrs.update(&imu_data.gyr, &imu_data.acc, &imu_data.mag);

        #[cfg(not(feature = "mag"))]
        let ahrs_est = ahrs.update_imu(&imu_data.gyr, &acceleration);

        match ahrs_est {
            Ok(q) => {
                let (roll,pitch,yaw) = q.euler_angles();
                let angle = Attitude::new( pitch,roll,yaw );
                let rate = Attitude::new(imu_data.gyr[1],imu_data.gyr[0],imu_data.gyr[2]);
                out_attitude_sense.publish_immediate((angle,rate));

                prev_quaternion = Some(*q);
            },
            Err(e) => {
                warn!("{} : Error in AHRS filter -> {}",TASK_ID,e)
            },
        }


    }
}

#[allow(unused)]
#[derive(Clone)]
enum EstimationError {
    MatrixSingular,
}