#pragma once

#include "../module/dataStructures.h"

class Kalman {
    public:

        // Provide new sensor readings

        void newAccel(VectorFloat accels, unsigned timestamp) {}
        void newGyro(VectorFloat gyro, uint32_t timestamp) {}
        void newMag(VectorFloat mag, uint32_t timestamp) {}

        void newGpsLocation(float lat, float lng, uint32_t timestamp) {}
        void newGpsAltitude(float alt, uint32_t timestamp) {}
        void newGpsVelocity(float vel,uint32_t timestamp) {}

        void newBaroAltitude(float baro,uint32_t timestamp) {}

        // Provide quadcopter actuations

        void newThrust(){}
        void newPitch(){}
        void newRoll(){}
        void newYaw(){}

        // Update contents of kalman filter

        void update(){}

        // Get estimated values

        float getLatitude(){return 0.0;}
        float getLongitude(){return 0.0;}

        VectorFloat getPosition () {return VectorFloat();}
        VectorFloat getVelocity () {return VectorFloat();}
        VectorFloat getAccel () {return VectorFloat();}

        VectorFloat getAngularVelocity () {return VectorFloat();}
        VectorFloat getAngularPositon () {return VectorFloat();}

    private:
};