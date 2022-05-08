#ifndef power_h
#define power_h

class Battery {
    public:
        Battery(float minVoltage,float maxVoltage);
        float getAdcReading();
        float getPercentage();
        float getVoltage();
        float getAdcReadingFast();
        bool isLow();
    private:
        float adcReading;
        float adcToVoltMult;
        float minimumVoltage;
        float acceptableVoltage;
        float maximumVoltage;
};

#endif
