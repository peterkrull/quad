#pragma once

#include <Arduino.h>

#if USE_MPU6050
#include "MPU6050/mpu6050.h"

class IMU {

    private:
        IMU_mpu6050 mpu;

        Quaternion quaternion;
        VectorFloat angular_rates;
        VectorFloat accelerations;

        bool new_reading;

    public:
        VectorFloat getAccel() { return mpu.getAccel(); }
        VectorFloat getGyro() {
            VectorFloat gyro = mpu.getGyro();
            #if RADIAN_DEGREES == false
            return gyro.getProduct(180.0/PI);
            #endif
            return gyro;
        }

        VectorFloat getPry() {
            #if RADIAN_DEGREES == false
            return mpu.getPry().getProduct(180/PI);
            #endif
            return mpu.getPry();
        }

        Quaternion getQuaternion() { return mpu.getQuaternion(); }
        bool isUpdated() { return mpu.isUpdated(); }
        bool initialize() { return mpu.initialize(); }
        bool statusGood() { return mpu.statusGood(); }
        unsigned long getTimestamp() {return mpu.getTimestamp(); }
};
#endif

#if HAS_MAGNETOMETER
VectorFloat magnetometer;
#endif

// class IMU_x {

//     private:
//         // IMU_mpu6050 mpu;
//         #if USE_MPU6050
//         IMU_mpu6050 mpu;
//         #endif

//         #if USE_MPU9250
//         IMU_mpu9250 mpu;
//         #endif

//         Quaternion quaternion;
//         VectorFloat angular_rates;
//         VectorFloat accelerations;

//         bool new_reading;

//     public:
//         VectorFloat getAccel() { return mpu.getAccel(); }
//         VectorFloat getGyro() {
//         VectorFloat gyro = mpu.getGyro();
        
//         #if RADIAN_DEGREES == false
//         return gyro.getProduct(180.0/PI);
//         #endif

//         return gyro;
//         }
//         bool isUpdated() { return mpu.isUpdated(); }
//         bool initialize() { return mpu.initialize(); }
//         unsigned long getTimestamp() {return mpu.getTimestamp(); }
//         #if HAS_MAGNETOMETER
//         VectorFloat getMagnetReading();
//         #endif
// };