use defmt::*;

use embassy_time::{Ticker, Duration};
use embassy_rp::{peripherals::I2C1, i2c};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as CSMutex, pubsub::Publisher};
use icm20948_async::{AccelerometerRange, AccelerometerDlp, AccelerometerUnit, GyroscopeRange, GyroscopeDlp, GyroscopeUnit, Icm20948, IcmError};

#[embassy_executor::task]
pub async fn imu_reader(
    i2c: I2cDevice<'static,CSMutex, i2c::I2c<'static, I2C1, i2c::Async>>,
    readings_ch: Publisher<'static, CSMutex, crate::ImuData, 1, 1, 1>,
    sample_time : Duration
) {
    info!("IMU_READER : start");

    // Create and await IMU object
    let imu_configured = Icm20948::new(i2c)
        // Configure accelerometer
        .acc_range(AccelerometerRange::Gs8)
        .acc_dlp(AccelerometerDlp::Hz111)
        .acc_unit(AccelerometerUnit::Mpss)
        // Configure gyroscope
        .gyr_range(GyroscopeRange::Dps1000)
        .gyr_dlp(GyroscopeDlp::Hz196)
        .gyr_unit(GyroscopeUnit::Rps)
        // Final initialization
        .set_address(0x69);

    #[cfg(not(feature = "mag"))]
    let imu_result = imu_configured.initialize_6dof().await;

    #[cfg(feature = "mag")]
    let imu_result = imu_configured.initialize_9dof().await;

    // Unpack IMU result safely and print error if necessary
    let mut imu = match imu_result {
        Ok(imu) => imu,
        Err(error) => {
            match error {
                IcmError::BusError(_)   => error!("IMU_READER : IMU encountered a communication bus error"),
                IcmError::ImuSetupError => error!("IMU_READER : IMU encountered an error during setup"),
                IcmError::MagSetupError => error!("IMU_READER : IMU encountered an error during mag setup")
            } return;
        }
    };

    // Calibrate gyroscope offsets using 100 samples
    info!("IMU_READER : Reading gyroscopes, keep still");
    let _gyr_cal = imu.gyr_calibrate(100).await.is_ok();

    // Magnetometer calibration scope
    #[cfg(feature = "mag")] {
        // Condition to start calibrating magnetometer
        info!("IMU_READER : Please rotate drone to calibrate magnetometer");
        loop {
            if let Ok(acc) = imu.read_acc().await {
                if acc[2] < 0. {
                    break;
                }
            }
        }

        use mag_calibrator_rs::MagCalibrator;
        let mut mag_cal = MagCalibrator::<30>::new().pre_scaler(200.);
        let mut ticker = Ticker::every(Duration::from_hz(10));
        loop {
            if let Ok(mag) = imu.read_mag().await {
                mag_cal.evaluate_sample_vec(mag);
                info!("MSD : {}", mag_cal.get_mean_distance());
                if mag_cal.get_mean_distance() > 0.035 {
                    if let Some((offset, scale)) = mag_cal.perform_calibration() {
                        imu.set_mag_calibration(offset, scale);
                        break;
                    }
                }
            }
            ticker.next().await;
        }
    }

    // Continuously read IMU data at constant sample rate
    let mut ticker = Ticker::every(sample_time);
    info!("IMU_READER : Entering main loop");
    loop {
        if let Ok(imu_data) = imu.read_all().await {
            readings_ch.publish_immediate(imu_data);
        }
        ticker.next().await;
    }
}
