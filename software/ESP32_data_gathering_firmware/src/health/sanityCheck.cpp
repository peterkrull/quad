#include "sanityCheck.h"

#include "../module/dataStructures.h"

sanityCheck::sanityCheck(){}

uint8_t sanityCheck::check(quadcopter_motors motors, allMpu mpuData){

    // Check for 90 deg pitch or roll
    if (abs(mpuData.ypr[1]) > PI/2 || abs(mpuData.ypr[2]) > PI/2) {return 0;}

    // Check for prolonged total motor power
    if (totalmotorsanity_lp.update(motors.M1+motors.M2+motors.M3+motors.M4)/4 > 1000) {return 0;}

    // Check for proloneged individual motor power
    if (M1sanity_lp.update(motors.M1) > 1500) {return 0;}
    if (M2sanity_lp.update(motors.M2) > 1500) {return 0;}
    if (M3sanity_lp.update(motors.M3) > 1500) {return 0;}
    if (M3sanity_lp.update(motors.M4) > 1500) {return 0;}

    // Check for abnormal acceleration values
    if (AAsanity_lp.update(mpuData.aaReal.getMagnitude()) > 5000) {return 0;}

    return 1;
}
