#ifndef dataStructures_h
#define dataStructures_h

#include <Arduino.h>
#include "DShotRMT.h"
#include "MPU6050_6Axis_MotionApps612.h"

struct quadcopter_motors{
  int16_t M1 = 0;
  int16_t M2 = 0;
  int16_t M3 = 0;
  int16_t M4 = 0;
};

struct pryt{
  float thrust = 0;
  float pitch = 0;
  float roll = 0;
  float yaw = 0;
};

struct motor {
  uint16_t speed = 48;
  DShotRMT dshot;
  bool reversed;
  bool rmt_init = false;
  uint8_t rmt_channel;
  uint8_t gpio_pin;
  bool standby = true;
  bool ready = false;
};

struct allMpu {
  bool dmpReady = false;  // set true if DMP init was successful
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer

  Quaternion q;        // [w, x, y, z]         quaternion container
  VectorInt16 aa;      // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity; // [x, y, z]            gravity vector
  VectorInt16 gyro;    // [x, y, z]            gyroscope vector
  float euler[3];      // [psi, theta, phi]    Euler angle container
  float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
};

#endif