
#define ESC_DSHOT true

// Dshot protocol
#if ESC_DSHOT

#include "actuator/dshot.h"

class xmotor {

    public:
        bool initialize() {}
        bool setSpeed() {}
        
        bool setDirection () {
            #if ALLOW_REVERSE_MOTOR_DIR
                return true;
            #else
                return false;
            #endif
        }
        

    private:


};

#endif
