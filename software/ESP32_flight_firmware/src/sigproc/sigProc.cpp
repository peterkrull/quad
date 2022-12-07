#include "sigProc.h"

low_pass::low_pass(float tau){
    xTau = tau;
    isInit = true;
}

bool low_pass::isInitialized() {
    return isInit;
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

    isInit = true;
}

bool PID::isInitialized() {
    return isInit;
}

float PID::update(float error){
    float xtime = micros()/1e6;
    double outsig = update(error,xtime-prev_time);
    prev_time = xtime;

    return outsig;
}

float PID::update(float error,float ro_min, float ro_max){
    float xtime = micros()/1e6;
    double outsig = update(error,xtime-prev_time, ro_min, ro_max);
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
        if (!(constrain(outsig, outlimMin, outlimMax) == outsig && antiwindup)) {
            integral += (xKi*error*dtime);
            outsig += integral;
        }
    }

    prev_error = error;

    // enforce output limit
    if (outlimMin != (float) NULL && outlimMax != (float) NULL) {
        return constrain(outsig, outlimMin, outlimMax);
    } else { return outsig; }
}

// With handling of roll-over
float PID::update(float error, float dtime, float ro_min, float ro_max){
    
    if ( error-prev_error < ro_min ){
        prev_error -= ro_max-ro_min;
        integral   -= ro_max-ro_min;
    } else if ( error-prev_error > ro_max ) {
        prev_error += ro_max-ro_min;
        integral   += ro_max-ro_min;
    }
  
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
        if (!(constrain(outsig, outlimMin, outlimMax) == outsig && antiwindup)) {
            integral += (xKi*error*dtime);
            outsig += integral;
        }
    }

    prev_error = error;

    // enforce output limit
    if (outlimMin != (float) NULL && outlimMax != (float) NULL) {
        return constrain(outsig, outlimMin, outlimMax);
    } else { return outsig; }
}

void PID::setOutputLimit(float min, float max){

    if (min != (float) NULL) {
        outlimMin = min;
    }

    if (max != (float) NULL) {
        outlimMax = max;
    }
}

void PID::setAntiwindup(bool set){
    antiwindup = set;
}

void PID::restart(){
    integral = 0;
    prev_time = micros();
}