#pragma once

#include "../module/dataStructures.h"

struct StateVector {
    VectorFloat pos;
    VectorFloat vel;
    VectorFloat acc;
} ;



struct StateCovarianceMatrix {
    float a [9][9];
} ;

class Kalman {
    public:

        // Provide new sensor readings

        void newAcc(VectorFloat acc, unsigned timestamp);
        void newPos(VectorFloat pos, unsigned timestamp);


        void newBaro(float baro,uint32_t timestamp) {}

        // Provide quadcopter actuations

        void newThrust(){}
        void newPitch(){}
        void newRoll(){}
        void newYaw(){}

        // Update contents of kalman filter

        StateVector predict(float Td);
        StateVector getStateVector() { return x_prime; }

        // Get estimated values

        float getLatitude(){return 0.0;}
        float getLongitude(){return 0.0;}

        VectorFloat getPosition () {return VectorFloat();}
        VectorFloat getVelocity () {return VectorFloat();}
        VectorFloat getAccel () {return VectorFloat();}

        VectorFloat getAngularVelocity () {return VectorFloat();}
        VectorFloat getAngularPositon () {return VectorFloat();}

    private:

        VectorFloat sigma_acc, sigma_pos;

        StateVector x_prime,x_pos;
        StateCovarianceMatrix p_prime,p_pos;
        bool newMeasurement;
};