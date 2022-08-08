#include "imu.h"
#include "../../module/helper_3dmath.h"

#ifdef USES_MPU6050

VectorFloat IMU::getAccelerations() {}

VectorFloat IMU::getGyroRates() {}

VectorFloat IMU::getMagnetReading() {}

bool IMU::isUpdated() {}

#endif