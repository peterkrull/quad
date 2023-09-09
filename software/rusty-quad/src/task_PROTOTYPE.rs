///*
/// This file can be used as a prototype (template) for new tasks 
///*/

use defmt::*;

static TASK_ID : &str = "PROTOTYPE(CHANGE THIS)";

#[embassy_executor::task]
pub async fn prototype(
    in_sg_attitude_sense: &'static crate::SgAttitudeSense,
) {

    info!("{} : Entering main loop",TASK_ID);
    loop {

        // Place looping code here, never break out of this loop!

    }
}
