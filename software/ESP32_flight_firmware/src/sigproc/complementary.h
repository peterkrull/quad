#ifndef complementary_h
#define complementary_h

#include "sigProc.h"

class complementaryFilter {
    public:
        complementaryFilter() {}
        complementaryFilter(float alpha);
        bool isInitialized();
        float update(float slow, float fast, float Td);
    private:
        bool isInit;
        float xAlpha, output;
};

class complementaryFilterAntiDrift {
    public:
        complementaryFilterAntiDrift(float alpha);
        float update(float slow, float fast, float Td);
    private:
        complementaryFilter comp;
        float init_val;
        low_pass lp_comp , lp_slow;
};

#endif
