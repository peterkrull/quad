/*

The communication channel type commonly used on this firmware is the Embassy PubSubChannel.

This module defines the channel types which interconnect all Embassy tasks, and act as the only
method for tasks to communicate values and states. By using the PubSubChannel type with a single
publisher and a capacity of 1, the channel will act like a 
*/


use embassy_sync::{
    pubsub::{PubSubChannel,Publisher,Subscriber},
    blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex}
};

// Take a value from a channel if it exists and assign it to the existing variable
pub fn try_assign_from_channel<const CAP : usize, const SUBS : usize, const PUBS : usize, M:RawMutex, T:Clone>
(channel : &mut Subscriber<M,T,CAP,SUBS,PUBS>, variable : &mut T) {
    if let Some(new_value) = channel.try_next_message_pure() { *variable = new_value }
}

type ChannelMutex = CriticalSectionRawMutex;

// Short-hand type alias for PubSubChannel
type Pub<T,const N: usize> = Publisher<'static,ChannelMutex,T,1,N,1>;
type Sub<T,const N: usize> = Subscriber<'static,ChannelMutex,T,1,N,1>;
type Ch<T,const N: usize> = PubSubChannel<ChannelMutex,T,1,N,1>;

const IMU_READING_NUM: usize = 1;
pub type ImuReadingType = crate::ImuData;
pub type ImuReadingPub = Pub<ImuReadingType,IMU_READING_NUM>;
pub type ImuReadingSub = Sub<ImuReadingType,IMU_READING_NUM>;
pub static IMU_READING : Ch<ImuReadingType,IMU_READING_NUM> = PubSubChannel::new();

const ATTITUDE_SENSE_NUM: usize = 2;
pub type AttitudeSenseType = (crate::task_attitude_controller::Attitude,crate::task_attitude_controller::Attitude);
pub type AttitudeSensePub = Pub<AttitudeSenseType,ATTITUDE_SENSE_NUM>;
pub type AttitudeSenseSub = Sub<AttitudeSenseType,ATTITUDE_SENSE_NUM>;
pub static ATTITUDE_SENSE : Ch<AttitudeSenseType,ATTITUDE_SENSE_NUM> = PubSubChannel::new();

const ATTITUDE_ACTUATE_NUM: usize = 1;
pub type AttitudeActuateType = crate::task_attitude_controller::Attitude;
pub type AttitudeActuatePub = Pub<AttitudeActuateType,ATTITUDE_ACTUATE_NUM>;
pub type AttitudeActuateSub = Sub<AttitudeActuateType,ATTITUDE_ACTUATE_NUM>;
pub static ATTITUDE_ACTUATE : Ch<AttitudeActuateType,ATTITUDE_ACTUATE_NUM> = PubSubChannel::new();

const ATTITUDE_INT_RESET_NUM: usize = 1;
pub type AttitudeIntResetType = bool;
pub type AttitudeIntResetPub = Pub<AttitudeIntResetType,ATTITUDE_INT_RESET_NUM>;
pub type AttitudeIntResetSub = Sub<AttitudeIntResetType,ATTITUDE_INT_RESET_NUM>;
pub static ATTITUDE_INT_RESET : Ch<AttitudeIntResetType,ATTITUDE_INT_RESET_NUM> = PubSubChannel::new();

const ATTITUDE_STAB_MODE_NUM: usize = 1;
pub type AttitudeStabModeType = crate::task_attitude_controller::StabilizationMode;
pub type AttitudeStabModePub = Pub<AttitudeStabModeType,ATTITUDE_STAB_MODE_NUM>;
pub type AttitudeStabModeSub = Sub<AttitudeStabModeType,ATTITUDE_STAB_MODE_NUM>;
pub static ATTITUDE_STAB_MODE : Ch<AttitudeStabModeType,ATTITUDE_STAB_MODE_NUM> = PubSubChannel::new();

const MOTOR_SPEED_NUM: usize = 1;
pub type MotorSpeedType = Option<(u16,u16,u16,u16)>;
pub type MotorSpeedPub = Pub<MotorSpeedType,MOTOR_SPEED_NUM>;
pub type MotorSpeedSub = Sub<MotorSpeedType,MOTOR_SPEED_NUM>;
pub static MOTOR_SPEED : Ch<MotorSpeedType,MOTOR_SPEED_NUM> = PubSubChannel::new();

const MOTOR_ARM_NUM: usize = 1;
pub type MotorArmType = bool;
pub type MotorArmPub = Pub<MotorArmType,MOTOR_ARM_NUM>;
pub type MotorArmSub = Sub<MotorArmType,MOTOR_ARM_NUM>;
pub static MOTOR_ARM : Ch<MotorArmType,MOTOR_ARM_NUM> = PubSubChannel::new();

const MOTOR_DIR_NUM: usize = 1;
pub type MotorDirType = (bool,bool,bool,bool);
pub type MotorDirPub = Pub<MotorDirType,MOTOR_DIR_NUM>;
pub type MotorDirSub = Sub<MotorDirType,MOTOR_DIR_NUM>;
pub static MOTOR_DIR : Ch<MotorDirType,MOTOR_DIR_NUM> = PubSubChannel::new();

const MOTOR_STATE_NUM: usize = 2;
pub type MotorStateType = crate::task_motor_governor::MotorState;
pub type MotorStatePub = Pub<MotorStateType,MOTOR_STATE_NUM>;
pub type MotorStateSub = Sub<MotorStateType,MOTOR_STATE_NUM>;
pub static MOTOR_STATE : Ch<MotorStateType,MOTOR_STATE_NUM> = PubSubChannel::new();

const SBUS_CMD_NUM: usize = 1;
pub type SbusCmdType = Result<crate::sbus_cmd::SbusCmd,crate::task_sbus_reader::SbusError>;
pub type SbusCmdPub = Pub<SbusCmdType,SBUS_CMD_NUM>;
pub type SbusCmdSub = Sub<SbusCmdType,SBUS_CMD_NUM>;
pub static SBUS_CMD : Ch<SbusCmdType,SBUS_CMD_NUM> = PubSubChannel::new();

const THRUST_ACTUATE_NUM: usize = 1;
pub type ThrustActuateType = f32;
pub type ThrustActuatePub = Pub<ThrustActuateType,THRUST_ACTUATE_NUM>;
pub type ThrustActuateSub = Sub<ThrustActuateType,THRUST_ACTUATE_NUM>;
pub static THRUST_ACTUATE : Ch<ThrustActuateType,THRUST_ACTUATE_NUM> = PubSubChannel::new();

const BLINKER_MODE_NUM: usize = 1;
pub type BlinkerModeType = crate::task_blinker::BlinkerMode;
pub type BlinkerModePub = Pub<BlinkerModeType,BLINKER_MODE_NUM>;
pub type BlinkerModeSub = Sub<BlinkerModeType,BLINKER_MODE_NUM>;
pub static BLINKER_MODE : Ch<BlinkerModeType,BLINKER_MODE_NUM> = PubSubChannel::new();

const FLIGHT_MODE_NUM: usize = 1;
pub type FlightModeType = crate::task_flight_detector::FlightState;
pub type FlightModePub = Pub<FlightModeType,FLIGHT_MODE_NUM>;
pub type FlightModeSub = Sub<FlightModeType,FLIGHT_MODE_NUM>;
pub static FLIGHT_MODE : Ch<FlightModeType,FLIGHT_MODE_NUM> = PubSubChannel::new();

/* DO NOT MODIFY PROTOTYPE
const CH_PROTOTYPE_X_NUM: usize = 1;
pub type ChPrototypeXType = bool;
pub type ChPrototypeXPub = Pub<ChPrototypeXType,1,CH_PROTOTYPE_X_NUM,1>;
pub type ChPrototypeXSub = Sub<ChPrototypeXType,1,CH_PROTOTYPE_X_NUM,1>;
pub static CH_PROTOTYPE_X : Ch<ChPrototypeXType,1,CH_PROTOTYPE_X_NUM,1> = PubSubChannel::new();
*/