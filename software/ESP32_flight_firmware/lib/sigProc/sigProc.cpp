#include "sigProc.h"

low_pass::low_pass(float tau){
    xTau = tau;
}

double low_pass::update(double input){
    uint32_t xtime = micros();
    double outsig = update(input,xtime-prev_time);
    prev_time = xtime;
    return outsig;
}

double low_pass::update(double input,uint32_t dtime){
    float a = dtime/(dtime+(xTau*1000000));
    output_val = output_val*(1-a)+input*a;
    return output_val;
}

void low_pass::restart(double value){
    output_val = value;
}

PID::PID(double Kp, double Ki, double Kd, float tau){
    xKp = Kp;
    xKi = Ki;
    xKd = Kd;
    if (tau > 0){
        lp = low_pass(tau);
        dlp = true;
    } else {
        dlp = false;
    }
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

    if (xKd && dlp){ // differential with lp
        differential = (xKd*1000000*(lp.update(error-prev_error)))/dtime;
        outsig += differential;
    } else if (xKd) { // differential
        differential = (xKd*1000000*(error-prev_error))/dtime;
        outsig += differential;
    }
    
    if (xKi){ // integral
        integral += (xKi*error*dtime/1000000);
        outsig += integral;
    }

    prev_error = error;

    return outsig;
}

