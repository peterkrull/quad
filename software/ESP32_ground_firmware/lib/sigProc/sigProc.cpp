#include "sigProc.h"

PID::PID(double Kp, double Ki, double Kd){
    xKp = Kp;
    xKi = Ki;
    xKd = Kd;
}

double PID::update(double error){
    uint32_t xtime = micros();
    double outsig = update(error,xtime-prev_time);
    prev_time = xtime;

    return outsig;
}

double PID::update(double error,uint32_t dtime){
    double outsig = 0;

    if (xKp){ // proportional
        outsig += xKp*error;
    }

    if (xKi){ // integral
        integral += xKi*error*dtime;
        outsig += integral;
    }

    if (xKd){ // differential
        outsig += xKd*(error-prev_error)/dtime;
    }

    prev_error = error;

    return outsig;
}