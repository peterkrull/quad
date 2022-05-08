#ifndef mpu6050_h
#define mpu6050_h

// Use 9-axis motion apps if 3D-magnetometer is available

#ifdef has3dMagnetometer
    #include "MPU6050_9Axis_MotionApps41.h"
#else
    #include "MPU6050_6Axis_MotionApps612.h"
#endif


#endif