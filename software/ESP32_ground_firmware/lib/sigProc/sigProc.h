#include <Arduino.h>

class PID{
    public:
        PID(double Kp = 0 , double Ki = 0 , double Kd = 0);
        double update(double error);
        double update(double error,uint32_t dtime);
        void restart();
    private:
        double xKp, xKi, xKd;
        uint32_t prev_time;
        double prev_error = 0;
        double integral = 0;
};