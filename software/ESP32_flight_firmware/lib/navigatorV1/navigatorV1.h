#include <Arduino.h>

#define commandStack 10

class navigator{
    public:
        navigator();
        command stack[commandStack];
        double update(double input);
    private:
        float xTau;
        uint32_t prev_time;
        double output_val;
};

class command{
    public:
        command();
        
};