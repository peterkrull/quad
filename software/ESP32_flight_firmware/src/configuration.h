#ifndef configuration_h
#define configuration_h

// Motherboard definition
#define BOARD_QUAD_V00

#define IMPORT_EASY_KIT true

#define HOVER_THRUST 730

#define RADIAN_DEGREES true

#define VOLT_MEA_CAL 1.00

#define X_IMU_ROTATION_REVERSE false
#define Y_IMU_ROTATION_REVERSE false
#define Z_IMU_ROTATION_REVERSE false
#define XY_SWAP_IMU_AXES false

#define ESC_DSHOT true
#define ESC_MULTISHOT false

#if ESC_DSHOT || ESC_MULTISHOT
    #define ALLOW_REVERSE_MOTOR_DIR false
#endif

#define LOW_BATTERY_HOMING true
#ifdef LOW_BATTERY_HOMING
    #define LOW_BATTERY_HOMING_VOLTAGE 13.5    
#endif

#define LOW_BATTERY_LAND true
#ifdef LOW_BATTERY_LAND
    #define LOW_BATTERY_LAND_VOLTAGE 12.5
#endif

#define UDP_TIMEOUT_MS 500

// Motor directions
#define REVERSE_MOTOR_M1 true
#define REVERSE_MOTOR_M2 false
#define REVERSE_MOTOR_M3 true
#define REVERSE_MOTOR_M4 false

// Serial communication
#define BAUDRATE 115200
#define ENABLE_SERIAL_PRINT true

// Controller limits

#define MAX_ROLL_PITCH_ANGLE 30
#define MAX_YAW_RATE 360

// Positioning limits

#define MAX_

// sanity check limits

#define GPS_INVALID_LAND true
#define GPS_INVALID_MS 10000

#define UDP_TIMEOUT_LAND true
#define UDP_TIMEOUT_MS 500

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
#define USE_THRUST_CONTROLLER
#define USE_PITCH_CONTROLLER
#define USE_ROLL_CONTROLLER
#define USE_YAW_CONTROLLER

// Velocity controllers
#define USE_VELOCITY_CONTROL
#define USE_POSITION_CONTROL

// ------------------------ //
// controller configuration //
// ------------------------ //

#define thrustInnerControllerType PID
#define thrustInnerController thrustInnerControllerType(1,0,0)

#define pitchInnerControllerType PID
#define pitchInnerController pitchInnerControllerType(10,0.333,0.05,0,true)

#define rollInnerControllerType PID
#define rollInnerController rollInnerControllerType(10,0.333,0.05,0,true)

#define yawInnerControllerType PID
#define yawInnerController yawInnerControllerType(1,0,0)

#define thrustControllerType PID
#define thrustController thrustControllerType(1,0,0)

#define pitchControllerType PID
#define pitchController pitchControllerType(12,0,0,0,true)

#define rollControllerType PID
#define rollController rollControllerType(12,0,0,0,true)

#define yawControllerType PID
#define yawController yawControllerType(1,0,0)

#endif