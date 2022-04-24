#ifndef configuration_h
#define configuration_h


// Motherboard definition
#define BOARD_QUAD_V01

#define hoverThrust             730

#define xImuRotationReverse     false
#define yImuRotationReverse     false
#define zImuRotationReverse     false
#define xySwapImuExes           false

#define reverseMotorDir         false

#define lowBatteryHome          true
#ifdef lowBatteryHome
    #define lowBatteryHomeV     13.5    
#endif

#define lowBatteryLand          true
#ifdef lowBatteryLand
    #define lowBatteryLandV     12.5
    #define lowBatteryThrust    (hoverThrust*0.9)
#endif

// sanity check limits

#define gpsInvalidLand          true
#define gpsInvalidMS            10000

#define udpTimeoutLand          true
#define udpTimeoutMS            500

#define maxPitchRollAngle       30
#define maxYawRate              360

#define maxRadiusFromTakeoff    20 
#define maxAltitudeFromTakeoff  10
#define maxMotorVelocity        1500

#define returnHome              true
#define panicOnSanityFail       false
#ifndef panicOnSanityFail
    #define descendOnSanityFail true
#endif

// Innermost controllers
#define usePitchController
#define useRollController
#define useYawController
#define useThrustController

// Velocity controllers
#define useVelocityControl
#define usePositionControl

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