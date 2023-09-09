#![feature(type_changing_struct_update)]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use embassy_futures as _;

use embassy_time::Duration;

const IMU_SAMPLE_TIME: Duration = Duration::from_hz(500);

#[cfg(not(feature = "mag"))]
type ImuData = icm20948_async::Data6Dof;
#[cfg(feature = "mag")]
type ImuData = icm20948_async::Data9Dof;

// Load module for cross-task channels
mod signals;

// Static i2c mutex for shared-bus functionality
use static_cell::StaticCell;
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as CSRMutex;
use embassy_rp::{i2c::{I2c,Async},peripherals::I2C1, gpio::Pin};
static SHARED_ASYNC_I2C : StaticCell<Mutex<CSRMutex, I2c<I2C1, Async>>> = StaticCell::new();

// Import task modules
mod task_blinker;
mod task_attitude_controller;
mod task_motor_governor;
mod task_imu_reader;
mod task_sbus_reader;
mod task_state_estimator;
mod task_commander;
mod task_motor_mixing;
mod task_flight_detector;

// Import misc modules
mod functions;
mod sbus_cmd;
mod imu;

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let p = embassy_rp::init(Default::default());

    // Create quad-motor PIO runner
    let quad_pio_motors = {
        use dshot_pio::dshot_embassy_rp::DshotPio;
        use embassy_rp::{pio::*,bind_interrupts,peripherals::PIO0};
        bind_interrupts!(struct Pio0Irqs {PIO0_IRQ_0 => InterruptHandler<PIO0>;});
        DshotPio::<4,_>::new(
            p.PIO0,
            Pio0Irqs,
            p.PIN_13,
            p.PIN_7,
            p.PIN_6,
            p.PIN_12,
            (52, 0)
        )
    };

    // Configure and setup sbus compatible uart RX connection
    let uart_rx_sbus = {
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
        use embassy_rp::{i2c,bind_interrupts};
        bind_interrupts!(struct I2c1Irqs {I2C1_IRQ => i2c::InterruptHandler<I2C1>;});
        let mut i2c_config = i2c::Config::default();
        i2c_config.frequency = 400_000; // High speed i2c
        let i2c = I2c::new_async(p.I2C1, p.PIN_11, p.PIN_10, I2c1Irqs, i2c_config);
        SHARED_ASYNC_I2C.init(Mutex::new(i2c))
    };

    // Share async i2c bus
    use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
    let i2c_bus_imu = I2cDevice::new(shared_i2c);

    // Spawning of system tasks

    use crate::task_commander::commander;
    spawner.must_spawn(commander(
        signals::SBUS_CMD.subscriber().unwrap(), 
        signals::MOTOR_STATE.subscriber().unwrap(),
        signals::ATTITUDE_SENSE.subscriber().unwrap(), 
        signals::FLIGHT_MODE.subscriber().unwrap(),
        signals::MOTOR_ARM.publisher().unwrap(), 
        signals::MOTOR_DIR.publisher().unwrap(),
        signals::THRUST_ACTUATE.publisher().unwrap(),
        signals::BLINKER_MODE.publisher().unwrap(),
        signals::ATTITUDE_INT_RESET.publisher().unwrap(),
        signals::ATTITUDE_STAB_MODE.publisher().unwrap(),
    ));

    use crate::task_sbus_reader::sbus_reader;
    spawner.must_spawn(sbus_reader(
        uart_rx_sbus,
        signals::SBUS_CMD.publisher().unwrap(),
        Duration::from_millis(500),
    ));

    use crate::task_imu_reader::imu_reader;
    spawner.must_spawn(imu_reader(
        i2c_bus_imu,
        signals::IMU_READING.publisher().unwrap(),
        IMU_SAMPLE_TIME
    ));

    use crate::task_state_estimator::state_estimator;
    spawner.must_spawn(state_estimator(
        signals::IMU_READING.subscriber().unwrap(), 
        signals::ATTITUDE_SENSE.publisher().unwrap(), 
        IMU_SAMPLE_TIME
    ));

    use crate::task_attitude_controller::attitude_controller;
    spawner.must_spawn(attitude_controller(
        signals::ATTITUDE_SENSE.subscriber().unwrap(),
        signals::ATTITUDE_INT_RESET.subscriber().unwrap(),
        signals::ATTITUDE_STAB_MODE.subscriber().unwrap(),
        signals::ATTITUDE_ACTUATE.publisher().unwrap(),
        IMU_SAMPLE_TIME
    ));

    use crate::task_motor_mixing::motor_mixing;
    spawner.must_spawn(motor_mixing(
        signals::THRUST_ACTUATE.subscriber().unwrap(),
        signals::ATTITUDE_ACTUATE.subscriber().unwrap(),
        signals::MOTOR_ARM.subscriber().unwrap(),
        signals::MOTOR_SPEED.publisher().unwrap(),
    ));

    use crate::task_motor_governor::motor_governor;
    spawner.must_spawn(motor_governor(
        signals::MOTOR_SPEED.subscriber().unwrap(),
        signals::MOTOR_DIR.subscriber().unwrap(),
        signals::MOTOR_STATE.publisher().unwrap(),
        quad_pio_motors,
        Duration::from_millis(100),
    ));

    use crate::task_blinker::blinker;
    spawner.must_spawn(blinker(
        signals::BLINKER_MODE.subscriber().unwrap(),
        p.PIN_25.degrade()
    ));

    use crate::task_flight_detector::flight_detector;
    spawner.must_spawn(flight_detector(
        signals::MOTOR_STATE.subscriber().unwrap(),
        signals::FLIGHT_MODE.publisher().unwrap(),
    ));

    // Ensure all signal subscribers are used
    assert!(signals::IMU_READING.subscriber().is_err());
    assert!(signals::ATTITUDE_SENSE.subscriber().is_err());
    assert!(signals::ATTITUDE_ACTUATE.subscriber().is_err());
    assert!(signals::ATTITUDE_INT_RESET.subscriber().is_err());
    assert!(signals::ATTITUDE_STAB_MODE.subscriber().is_err());
    assert!(signals::MOTOR_SPEED.subscriber().is_err());
    assert!(signals::MOTOR_ARM.subscriber().is_err());
    assert!(signals::MOTOR_DIR.subscriber().is_err());
    assert!(signals::MOTOR_STATE.subscriber().is_err());
    assert!(signals::SBUS_CMD.subscriber().is_err());
    assert!(signals::THRUST_ACTUATE.subscriber().is_err());
    assert!(signals::BLINKER_MODE.subscriber().is_err());

}
