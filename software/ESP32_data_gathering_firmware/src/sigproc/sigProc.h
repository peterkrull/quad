#include <Arduino.h>

#ifndef sigProg_h
#define sigProg_h

class low_pass{
    public:
        low_pass(float tau);
        double update(double input);
        double update(double input,uint32_t dtime);
        void restart(double value);
    private:
        float xTau;
        uint32_t prev_time;
        double output_val;
};

class PID{
    public:
        PID(float Kp = 0 , float Ki = 0 , float Kd = 0 , float tau = 0, bool ideal = false);
        float update(float error);
        float update(float error,float dtime);
        void setOutputLimit(float min, float max);
        void setOutputLimEn(bool setMin,bool setMax);
        void setAntiwindup(bool set);
        void restart();
    private:
        float xKp, xKi, xKd;
        bool isOutlimMin = false;
        bool isOutlimMax = false;
        float outlimMin = 0;
        float outlimMax = 0;
        low_pass lp = low_pass(0);
        bool dlp;
        bool antiwindup = false;
        float prev_time;
        float prev_error = 0;
        float integral = 0;
};

float limiter(float in, float min, float max);

#endif