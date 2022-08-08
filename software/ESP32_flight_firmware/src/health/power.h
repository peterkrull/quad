#pragma once

class Battery {
    public:
        Battery(float minVoltage,float maxVoltage,int pin);
        float getAdcReading();
        float getPercentage();
        float getVoltage();
        float getAdcReadingFast();
        bool isLow();
    private:
        int vPin;
        float adcReading;
        float adcToVoltMult;
        
        float minimumVoltage, minTakeoffVoltage;
        float maximumVoltage;
};

