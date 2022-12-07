#ifdef pin_VBAT

Battery::Battery(float minVoltage,float maxVoltage,int pin){
    minimumVoltage = minVoltage;
    maximumVoltage = maxVoltage;
    vPin = pin;

    minTakeoffVoltage = minimumVoltage + (maximumVoltage - minimumVoltage) * 0.25;

    pinMode(pin_VBAT,INPUT);
}

float Battery::getAdcReading(){

}

float Battery::getPercentage(){}
float Battery::getVoltage(){}
float Battery::getAdcReadingFast(){}


bool Battery::isLow(){
    if (getPercentage() < minTakeoffVoltage) {
        return true;
    } else { return false;}
}

#endif