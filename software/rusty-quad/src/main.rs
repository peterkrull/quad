#![feature(type_alias_impl_trait)]
#![no_std]
#![no_main]
#![allow(unused)]

use core::f32::consts::PI;

use defmt::*;
use defmt_rtt as _;
use functions::mapf;
use panic_probe as _;

// Hopefully we can get rid of these
use rp2040_hal::gpio::FunctionUart;
use rp2040_hal::gpio::InputOverride::Invert;
use rp2040_hal::{pac, sio::Sio};

use embassy_executor::Spawner;
use embassy_futures as _;
use embassy_rp::i2c;
use embassy_rp::peripherals::{I2C1, UART1};
use embassy_rp::uart::{self, Async, Config, DataBits, Parity, StopBits, UartRx};
use embassy_rp::{self as _, interrupt};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{DynSubscriber, PubSubChannel, Publisher};
use embassy_time::{with_timeout, Duration, Instant, Ticker, Timer};

use ahrs::{Ahrs, Madgwick};

use icm20948_async::*;
use quad_dshot_pio::QuadDshotPio;
use sbus::SBusPacketParser;

mod functions;
mod sbus_cmd;

use mag_calibrator_rs::MagCalibrator;

const SAMPLE_TIME: Duration = Duration::from_hz(250);

// Short-hand type definitions
type U16x4 = (u16, u16, u16, u16);
type ImuData = icm20948_async::Data6Dof;
type SbusDur = (Option<sbus_cmd::SbusCmd>, Duration);
type CSMutex = CriticalSectionRawMutex;

// Cross-task channels
static CH_MOTOR_THROTTLE: PubSubChannel<CSMutex, U16x4, 1, 4, 1> = PubSubChannel::new();
static CH_IMU_READINGS: PubSubChannel<CSMutex, ImuData, 1, 4, 1> = PubSubChannel::new();
static CH_MOTOR_INIT: PubSubChannel<CSMutex, bool, 1, 4, 1> = PubSubChannel::new();
static CH_SBUS_CMD: PubSubChannel<CSMutex, SbusDur, 1, 4, 1> = PubSubChannel::new();

fn motor_mixing(thrust: f32, pitch: f32, roll: f32, yaw: f32, min: f32, max: f32) -> U16x4 {
    (
        (thrust + pitch + roll + yaw).clamp(min, max) as u16,
        (thrust - pitch - roll + yaw).clamp(min, max) as u16,
        (thrust + pitch - roll - yaw).clamp(min, max) as u16,
        (thrust - pitch + roll - yaw).clamp(min, max) as u16,
    )
}


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // This has to be done before configuring pins using Embassy
    let mut pac = pac::Peripherals::take().unwrap();
    let sio = Sio::new(pac.SIO);
    let pins = rp2040_hal::gpio::bank0::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Setup async I2C communication
    let irq = interrupt::take!(I2C1_IRQ);
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = 400_000;
    let i2c = i2c::I2c::new_async(p.I2C1, p.PIN_11, p.PIN_10, irq, i2c_config);

    // Setup sbus compatible uart connection
    let mut sbus_uart_config = Config::default();
    sbus_uart_config.baudrate = 100000;
    sbus_uart_config.data_bits = DataBits::DataBits8;
    sbus_uart_config.stop_bits = StopBits::STOP2;
    sbus_uart_config.parity = Parity::ParityEven;
    let mut sbus_rx_pin = pins.gpio9.into_mode::<FunctionUart>();
    let sbus_uart = uart::UartRx::new(p.UART1, p.PIN_9, p.DMA_CH0, sbus_uart_config);
    sbus_rx_pin.set_input_override(Invert);

    // Create quad-motor runner
    let motors = QuadDshotPio::new(
        pac.PIO0,
        &mut pac.RESETS,
        pins.gpio13.into_mode(),
        pins.gpio7.into_mode(),
        pins.gpio6.into_mode(),
        pins.gpio12.into_mode(),
        (52, 0),
    );

    // Spawning of forever-tasks

    spawner.must_spawn(imu_reader(i2c, CH_IMU_READINGS.publisher().unwrap()));

    spawner.must_spawn(sbus_parser(
        sbus_uart,
        CH_SBUS_CMD.publisher().unwrap(),
        Duration::from_millis(500),
    ));

    spawner.must_spawn(motor_governor(
        motors,
        CH_MOTOR_THROTTLE.dyn_subscriber().unwrap(),
        CH_MOTOR_INIT.dyn_subscriber().unwrap(),
        Duration::from_millis(100),
    ));

    spawner.must_spawn(control_loop(
        CH_MOTOR_THROTTLE.publisher().unwrap(),
        CH_MOTOR_INIT.publisher().unwrap(),
        CH_SBUS_CMD.dyn_subscriber().unwrap(),
    ));
}

#[embassy_executor::task]
async fn control_loop(
    motors: Publisher<'static, CSMutex, U16x4, 1, 4, 1>,
    mut motors_init: Publisher<'static, CSMutex, bool, 1, 4, 1>,
    mut sbus_sub: DynSubscriber<'static, SbusDur>,
) {
    let mut user_command = sbus_cmd::SbusCmd::default();
    let mut imu_data = CH_IMU_READINGS.dyn_subscriber().unwrap();

    // Filter to obtain quaternion from acc, gyr and mag
    let mut ahrs = Madgwick::new(SAMPLE_TIME.as_micros() as f32 / 1e6, 0.02);

    let mut yaw_integrator = 0.;

    info!("CONTROL_LOOP : Entering main loop");
    loop {
        // Received imu messages
        let data = imu_data.next_message_pure().await;
        if let Ok(q) = ahrs.update_imu(&data.gyr, &data.acc) {
            let (roll, pitch, yaw) = q.euler_angles();

            // Check that sbus command is sane
            sbus_sanity(&mut user_command, &mut motors_init, &mut sbus_sub);

            // Reset integrator if previously non-armed
            if !user_command.sw_b.is_active() {
                yaw_integrator = yaw;
            }

            // Controllers (all are proportional + cascaded)
            let p = (user_command.pitch - pitch) * 7.;
            let pi = (p - data.gyr[1]) * 60.;

            let r = (user_command.roll - roll) * 7.;
            let ri = (r - data.gyr[0]) * 60.;

            yaw_integrator -= user_command.yaw * SAMPLE_TIME.as_micros() as f32 / 1e6;
            yaw_integrator = functions::circular(yaw_integrator, -PI, PI);
            let y = (yaw_integrator - yaw) * 10.;
            let yi = (y - data.gyr[2]) * 80.;

            // Set throttle behavior based on switch C
            let t = match user_command.sw_c {
                sbus_cmd::TwiSwitch::Idle => mapf(user_command.thrust, 0., 1., 80., 1000.),
                sbus_cmd::TwiSwitch::Middle => mapf(user_command.thrust, 0., 1., 250., 1500.),
                sbus_cmd::TwiSwitch::Active => mapf(user_command.thrust, 0., 1., 250., 2047.),
            };

            let command = motor_mixing(t, pi, ri, yi, 70., 2047.);

            motors.publish_immediate(command);
        }
    }
}

#[embassy_executor::task]
async fn imu_reader(
    i2c: i2c::I2c<'static, I2C1, i2c::Async>,
    readings_ch: Publisher<'static, CSMutex, ImuData, 1, 4, 1>,
) {
    info!("IMU_READER : start");

    // Create and await IMU object
    let imu_result = Icm20948::new(i2c)
        // Configure accelerometer
        .acc_range(AccelerometerRange::Gs8)
        .acc_dlp(AccelerometerDlp::Hz111)
        .acc_unit(AccelerometerUnit::Mpss)
        // Configure gyroscope
        .gyr_range(GyroscopeRange::Dps1000)
        .gyr_dlp(GyroscopeDlp::Hz196)
        .gyr_unit(GyroscopeUnit::Rps)
        // Final initialization
        .set_address(0x69)
        .initialize_6dof()
        .await;

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

    // // Condition to start calibrating magnetometer
    // info!("IMU_READER : Please rotate drone to calibrate magnetometer");
    // loop {
    //     if let Ok(acc) = imu.read_acc().await {
    //         if acc[2] < 0. {
    //             break;
    //         }
    //     }
    // }

    // // Magnetometer calibration scope
    // {
    //     let mut mag_cal = MagCalibrator::<30>::new().pre_scaler(200.);
    //     let mut ticker = Ticker::every(Duration::from_hz(10));
    //     loop {
    //         if let Ok(mag) = imu.read_mag().await {
    //             mag_cal.evaluate_sample_vec(mag);
    //             info!("MSD : {}", mag_cal.get_mean_distance());
    //             if mag_cal.get_mean_distance() > 0.035 {
    //                 if let Some((offset, scale)) = mag_cal.perform_calibration() {
    //                     imu.set_mag_calibration(offset, scale);
    //                     break;
    //                 }
    //             }
    //         }
    //         ticker.next().await;
    //     }
    // }

    let mut ticker = Ticker::every(SAMPLE_TIME);
    info!("IMU_READER : Entering main loop");
    loop {
        if let Ok(imu_data) = imu.read_all().await {
            readings_ch.publish_immediate(imu_data)
        }
        ticker.next().await;
    }
}

fn sbus_sanity(
    sbus_msg_out: &mut sbus_cmd::SbusCmd,
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

#[embassy_executor::task]
async fn sbus_parser(
    mut rx: UartRx<'static, UART1, Async>,
    ch_sbus_cmd: Publisher<'static, CSMutex, SbusDur, 1, 4, 1>,
    timeout: Duration,
) {
    let mut parser = SBusPacketParser::new();
    let mut prev_parse_time = Instant::now();

    // A buffer length of 1 may be slower, but perhaps also reduces
    // the risk of the buffer coming out of sync with the message..
    let mut read_buffer = [0; 1];

    info!("SBUS_PARSER : Entering main loop");
    loop {
        if Ok(Ok(())) == with_timeout(timeout, rx.read(&mut read_buffer)).await {
            parser.push_bytes(&read_buffer);
            if let Some(packet) = parser.try_parse() {
                let parse_time = Instant::now();
                let reformat = sbus_cmd::convert(&packet);
                ch_sbus_cmd
                    .publish_immediate((reformat, parse_time.duration_since(prev_parse_time)));
                prev_parse_time = parse_time;
            }
        } else {
            ch_sbus_cmd.publish_immediate((None, prev_parse_time.elapsed()));
        }
    }
}

#[embassy_executor::task]
async fn motor_governor(
    mut motors: QuadDshotPio<pac::PIO0>,
    mut set_speeds: DynSubscriber<'static, U16x4>,
    mut arm_motors: DynSubscriber<'static, bool>,
    timeout: Duration,
) {
    loop {
        // Wait for signal to arm motors
        while !arm_motors.next_message_pure().await {}

        // Send minimum throttle for a few seconds to let
        // the ESCs know that they should be armed
        info!("MOTOR_GOVERNOR : Initializing motors");
        Timer::after(Duration::from_millis(500)).await;
        for _i in 0..50 {
            motors.throttle_minimum();
            Timer::after(Duration::from_millis(50)).await;
        }

        // Set motor directions for the four motors
        info!("MOTOR_GOVERNOR : Setting motor directions");
        for _i in 0..10 {
            motors.reverse((true, true, false, false));
            Timer::after(Duration::from_millis(50)).await;
        }

        info!("MOTOR_GOVERNOR : Entering main loop");
        loop {
            // Break if commanded to disarm
            if Some(false) == arm_motors.try_next_message_pure() {
                warn!("MOTOR_GOVERNOR : Disarming motors -> commanded by parser");
                break;
            }

            // Retrieve speeds from channel and transmit, break on timeout
            if let Ok(speeds) = with_timeout(timeout, set_speeds.next_message_pure()).await {
                motors.throttle_clamp(speeds);
            } else {
                warn!("MOTOR_GOVERNOR : Disarming motors -> message timeout");
                break;
            }
        }
    }
}
