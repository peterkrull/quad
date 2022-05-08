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

PID::PID(float Kp, float Ki, float Kd, float tau, bool ideal){
    if (ideal){
        xKp = Kp;
        xKi = Ki*Kp;
        xKd = Kd*Kp;  
    } else {
        xKp = Kp;
        xKi = Ki;
        xKd = Kd;     
    }
    if (tau > 0){
        lp = low_pass(tau);
        dlp = true;
    } else {
        dlp = false;
    }
}

float PID::update(float error){
    float xtime = micros()/1e6;
    double outsig = update(error,xtime-prev_time);
    prev_time = xtime;

    return outsig;
}

float PID::update(float error,float dtime){
    double outsig = 0;

    if (xKp){ // proportional
        outsig += xKp*error;
    }

    if (xKd) { // derivative
        float delta = error-prev_error;
        if (dlp) {delta = lp.update(delta);}
        float derivative = (xKd*(delta))/dtime;
        outsig += derivative;
    }
    
    if (xKi){ // integral
        if (limiter(outsig, outlimMin, outlimMax) == outsig || !antiwindup){
            integral += (xKi*error*dtime);
            outsig += integral;
        }
    }

    prev_error = error;

    return limiter(outsig, outlimMin, outlimMax);
}

void PID::setOutputLimit(float min, float max){

    if (isOutlimMin) {
        outlimMin = min;
    }

    if (isOutlimMax) {
        outlimMax = max;
    }
}

void PID::setOutputLimEn(bool setMin,bool setMax){
    isOutlimMin = setMin;
    isOutlimMax = setMax;
}

void PID::setAntiwindup(bool set){
    antiwindup = set;
}

void PID::restart(){
    integral = 0;
    prev_time = micros();
}

float limiter(float in, float min, float max){
    if ( in > max) return max;
    else if ( in < min) return min;
    else return in;
}