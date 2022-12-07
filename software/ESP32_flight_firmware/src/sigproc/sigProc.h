#include <Arduino.h>

#ifndef sigProg_h
#define sigProg_h

class low_pass{
    public:
        low_pass(){}
        low_pass(float tau);
        bool isInitialized();
        double update(double input);
        double update(double input,uint32_t dtime);
        void restart(double value);
    private:
        float xTau;
        bool isInit = false;
        uint32_t prev_time;
        double output_val;
};

class PID{
    public:
        PID() {}
        PID(float Kp , float Ki = 0 , float Kd = 0 , float tau = 0, bool ideal = false);
        float update(float error);
        float update(float error, float dtime);
        float update(float error, float ro_min, float ro_max);
        float update(float error, float dtime, float ro_min, float ro_max);
        bool isInitialized();
        void setOutputLimit(float min, float max);
        void setAntiwindup(bool set);
        void restart();
    private:
        float xKp, xKi, xKd;
        float outlimMin = (float) NULL;
        float outlimMax = (float) NULL;
        low_pass lp = low_pass(0);
        bool dlp;
        bool antiwindup = false;
        bool isInit = false;
        float prev_time;
        float prev_error = 0;
        float integral = 0;
};

float limiter(float in, float min, float max);

#endif