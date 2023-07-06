#![feature(type_changing_struct_update)]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use embassy_futures as _;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as CSMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::Duration;

mod functions;
mod sbus_cmd;

mod imu;

const IMU_SAMPLE_TIME: Duration = Duration::from_hz(500);

// Short-hand type definitions
type U16x4 = (u16, u16, u16, u16);
#[cfg(not(feature = "mag"))]
type ImuData = icm20948_async::Data6Dof;
#[cfg(feature = "mag")]
type ImuData = icm20948_async::Data9Dof;

type SbusDur = (Option<sbus_cmd::SbusCmd>, Duration);

// Cross-task channels
static CH_MOTOR_THROTTLE: PubSubChannel<CSMutex, U16x4, 1, 1, 1> = PubSubChannel::new();
static CH_IMU_READINGS: PubSubChannel<CSMutex, ImuData, 1, 1, 1> = PubSubChannel::new();
static CH_MOTOR_INIT: PubSubChannel<CSMutex, bool, 1, 1, 1> = PubSubChannel::new();
static CH_SBUS_CMD: PubSubChannel<CSMutex, SbusDur, 1, 1, 1> = PubSubChannel::new();

// Static i2c mutex for shared-bus functionality
use embassy_rp::{i2c::{I2c,Async},peripherals::{I2C1}};
use static_cell::StaticCell;
use embassy_sync::mutex::Mutex;
static SHARED_ASYNC_I2C : StaticCell<Mutex<CSMutex, I2c<I2C1, Async>> > = StaticCell::new();

mod task_control_loop;
mod task_motor_governor;
mod task_imu_reader;
mod task_sbus_reader;
mod task_blinker;

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let p = embassy_rp::init(Default::default());

    // Configure and setup sbus compatible uart RX connection
    let sbus_uart = {
        use embassy_rp::{uart::*,bind_interrupts,peripherals::UART1};
        bind_interrupts!(struct Uart1Irqs {UART1_IRQ => InterruptHandler<UART1>;});
        let mut sbus_uart_config = Config::default();
        sbus_uart_config.baudrate = 100_000;
        sbus_uart_config.data_bits = DataBits::DataBits8;
        sbus_uart_config.stop_bits = StopBits::STOP2;
        sbus_uart_config.parity = Parity::ParityEven;
        sbus_uart_config.invert_rx = true;
        UartRx::new(p.UART1, p.PIN_9,Uart1Irqs, p.DMA_CH0, sbus_uart_config)
    };

    // Configure and setup shared async I2C communication
    let shared_i2c = {
        use embassy_rp::{i2c::{Config,I2c,InterruptHandler},peripherals::I2C1,bind_interrupts};
        use embassy_sync::mutex::Mutex;
        bind_interrupts!(struct I2c1Irqs {I2C1_IRQ => InterruptHandler<I2C1>;});
        let mut i2c_config = Config::default();
        i2c_config.frequency = 400_000; // High speed i2c
        let i2c = I2c::new_async(p.I2C1, p.PIN_11, p.PIN_10, I2c1Irqs, i2c_config);
        SHARED_ASYNC_I2C.init(Mutex::new(i2c))
    };

    // Share async i2c bus
    use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
    let imu_i2c_bus = I2cDevice::new(shared_i2c);

    // Create quad-motor runner
    let motors = {
        use quad_dshot_pio::dshot_embassy_rp::QuadDshotPio;
        QuadDshotPio::new(p.PIO0,p.PIN_13,p.PIN_7,p.PIN_6,p.PIN_12,(52, 0))
    };

    // Spawning of system tasks

    use crate::task_imu_reader::imu_reader;
    spawner.must_spawn(imu_reader(
        imu_i2c_bus,
        CH_IMU_READINGS.publisher().unwrap(),
        IMU_SAMPLE_TIME
    ));

    use crate::task_sbus_reader::sbus_reader;
    spawner.must_spawn(sbus_reader(
        sbus_uart,
        CH_SBUS_CMD.publisher().unwrap(),
        Duration::from_millis(500),
    ));

    use crate::task_motor_governor::motor_governor;
    spawner.must_spawn(motor_governor(
        motors,
        CH_MOTOR_THROTTLE.dyn_subscriber().unwrap(),
        CH_MOTOR_INIT.dyn_subscriber().unwrap(),
        (true,true,false,false),
        Duration::from_millis(100),
    ));

    use crate::task_control_loop::control_loop;
    spawner.must_spawn(control_loop(
        CH_MOTOR_THROTTLE.publisher().unwrap(),
        CH_MOTOR_INIT.publisher().unwrap(),
        CH_IMU_READINGS.dyn_subscriber().unwrap(),
        CH_SBUS_CMD.dyn_subscriber().unwrap(),
        IMU_SAMPLE_TIME
    ));

}
