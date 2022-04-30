#ifndef sanityCheck_h
#define sanityCheck_h

#include <Arduino.h>
#include "../sigproc/sigProc.h"
#include "../module/dataStructures.h"

class sanityCheck{

    public:
        sanityCheck(/* args */);
        uint8_t check(quadcopter_motors motors, allMpu mpuData);
    private:
        low_pass totalmotorsanity_lp = low_pass(0.5);
        low_pass M1sanity_lp = low_pass(0.3);
        low_pass M2sanity_lp = low_pass(0.3);
        low_pass M3sanity_lp = low_pass(0.3);
        low_pass M4sanity_lp = low_pass(0.3);
        low_pass AAsanity_lp = low_pass(0.1);
};

#endif