#include <Arduino.h>

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
        PID(double Kp = 0 , double Ki = 0 , double Kd = 0 , float tau = 0, bool ideal = false);
        double update(double error);
        double update(double error,uint32_t dtime);
        void setOutputLimit(double min, double max);
        void setAntiwindup(bool set);
        void restart();
    private:
        double xKp, xKi, xKd;
        double outlimMax = NULL;
        double outlimMin = NULL;
        low_pass lp = low_pass(0);
        bool dlp;
        bool antiwindup = false;
        uint32_t prev_time;
        double prev_error = 0;
        double differential = 0;
        double integral = 0;
};

double limiter(double in, double min, double max);
