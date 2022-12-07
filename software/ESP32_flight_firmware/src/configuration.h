#pragma once

#ifndef configuration_h
#define configuration_h

// Motherboard definition
#define BOARD_QUAD_V00

#define HOVER_THRUST 0.365

#define RADIAN_DEGREES true

#define VOLT_MEA_CAL 1.00

#define X_IMU_ROTATION_REVERSE false
#define Y_IMU_ROTATION_REVERSE false
#define Z_IMU_ROTATION_REVERSE false
#define XY_SWAP_IMU_AXES false

#define SBUS_ENABLE true

#define ESC_DSHOT true
#define ESC_MULTISHOT false

#if ESC_DSHOT || ESC_MULTISHOT
    #define ALLOW_REVERSE_MOTOR_DIR true
#else
    #define ALLOW_REVERSE_MOTOR_DIR false
#endif

// battery config

#define MONITOR_BATTERIES true

#define BATTERY_CELLS 4
#define BATTERY_VMIN BATTERY_CELLS*3.0
#define BATTERY_VMAX BATTERY_CELLS*4.2

#define LOW_BATTERY_HOMING true
#ifdef LOW_BATTERY_HOMING
    #define LOW_BATTERY_HOMING_VOLTAGE 13.5    
#endif

#define LOW_BATTERY_LAND true
#ifdef LOW_BATTERY_LAND
    #define LOW_BATTERY_LAND_VOLTAGE 12.5
#endif

#define M1_REVERSE_DIR true
#define M2_REVERSE_DIR true
#define M3_REVERSE_DIR false
#define M4_REVERSE_DIR false

// Serial communication
#define BAUDRATE 500000
#define ENABLE_SERIAL_PRINT true

// Controller limits

#define MAX_ROLL_PITCH_ANGLE 30
#define MAX_YAW_RATE 360

#define MAX_MOTOR_DIFF 0.2

// Positioning limits

#define MAX_DISTANCE_FROM_HOME 20

// sanity check limits

#define GPS_INVALID_LAND true
#define GPS_INVALID_MS 10000

#define UDP_TIMEOUT_LAND true
#define UDP_TIMEOUT_MS 200

#define SANITY_ROLL_PITCH_ANGLE 50
#define SANITY_YAW_RATE 720

#define MAX_RADIUS_FROM_TAKEOFF 20 
#define MAX_ALTITUDE_FROM_TAKEOFF 10
#define MAX_MOTOR_VELOCITY 1500

#define DO_SANITY_CHECK true

#if DO_SANITY_CHECK
    #define ON_SANITY_FAIL_DESCEND true
    #define ON_SANITY_FAIL_PANIC false
    #define ON_SANITY_FAIL_HOME false
#endif

// Innermost controllers
#define USE_THRUST_CONTROLLER true
#define USE_PITCH_CONTROLLER true
#define USE_ROLL_CONTROLLER true
#define USE_YAW_CONTROLLER true

// Velocity controllers
#define USE_VELOCITY_CONTROL
#define USE_POSITION_CONTROL

// ------------------------ //
// controller configuration //
// ------------------------ //

// Thrust
#define CON_THRUST_INN_t PID
#define CON_THRUST_INN_c CON_THRUST_INN_t(1,0,0)
#define CON_THRUST_OUT_t PID
#define CON_THRUST_OUT_c CON_THRUST_OUT_t(1,0,0)

// Pitch
#define CON_PITCH_INN_t PID
#define CON_PITCH_INN_c CON_PITCH_INN_t(50,0.0,0.01,0,true)
#define CON_PITCH_OUT_t PID
#define CON_PITCH_OUT_c CON_PITCH_OUT_t(5,0,0.001,0,true)

// Roll
#define CON_ROLL_INN_t PID
#define CON_ROLL_INN_c CON_ROLL_INN_t(50,0.0,0.01,0,true)
#define CON_ROLL_OUT_t PID
#define CON_ROLL_OUT_c CON_ROLL_OUT_t(5,0,0.001,0,true)

// Yaw
#define CON_YAW_INN_t PID
#define CON_YAW_INN_c CON_YAW_INN_t(30,0,0)
#define CON_YAW_OUT_t PID
#define CON_YAW_OUT_c CON_YAW_OUT_t(5,0,0)

#endif // configuration_h

// Warning : Be careful not to remove this check
#include "config_sanity.h"