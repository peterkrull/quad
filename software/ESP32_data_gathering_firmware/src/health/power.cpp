#include <Arduino.h>
#include "power.h"
#include "../includes.h"

#ifdef pin_VBAT

Battery::Battery(float minVoltage,float maxVoltage){
    minimumVoltage = minVoltage;
    maximumVoltage = maxVoltage;
    pinMode(pin_VBAT,INPUT);
}

float Battery::getAdcReading(){

}

float Battery::getPercentage(){}
float Battery::getVoltage(){}
float Battery::getAdcReadingFast(){}
bool Battery::isLow(){}

#endif