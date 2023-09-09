///*
/// This file can be used as a prototype (template) for new tasks 
///*/

use defmt::*;
use embassy_time::{Ticker, Duration};
use crate::signals;

use crate::task_motor_governor::{MotorState, ArmedState};

static TASK_ID : &str = "FLIGHT_DETECTOR";

#[embassy_executor::task]
pub async fn flight_detector(
    mut in_motor_state: signals::MotorStateSub,
    out_flight_mode: signals::FlightModePub,
) {

    let mut flight_state = FlightState::Ground;

    let mut ticker = Ticker::every(Duration::from_hz(20));
    
    info!("{} : Entering main loop",TASK_ID);
    loop {

        let motor_state = in_motor_state.next_message_pure().await;


        match flight_state {
            FlightState::Ground => {
                if let MotorState::Armed(ArmedState::Running(m)) = motor_state {
                    let z_thrust = (m.0 + m.1 + m.2 + m.3)/4; 
                    if z_thrust > 750 {
                        flight_state = FlightState::Flying;
                        out_flight_mode.publish_immediate(flight_state.clone());
                        info!("{} : Changing flight mode to: {}",TASK_ID,flight_state);
                    }
                }
            },
            FlightState::Flying => {
                if let MotorState::Disarmed(_) = motor_state {
                    flight_state = FlightState::Ground;
                    out_flight_mode.publish_immediate(flight_state.clone());
                    info!("{} : Changing flight mode to: {}",TASK_ID,flight_state);
                }
            },
        }

        ticker.next().await;
    }
}

#[derive(Clone,Copy,PartialEq,Format)]
pub enum FlightState {
    Ground,
    Flying
}