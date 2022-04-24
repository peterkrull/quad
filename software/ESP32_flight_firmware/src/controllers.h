#include "dataStructures.h"
#include "configuration.h"
#include "sigProc.h"
#include "boards.h"

struct conStruct {
    #ifdef useThrustController
        thrustInnerControllerType pid_thrust = thrustController;
        #ifdef thrustInnerController
        thrustInnerControllerType pid_vthrust = thrustInnerController;
        #endif
    #endif

    #ifdef usePitchController
        pitchControllerType  pid_pitch = pitchController;
        #ifdef pitchInnerController
        pitchInnerControllerType pid_vpitch = pitchInnerController;
        #endif
    #endif

    #ifdef useRollController
        rollControllerType pid_roll = rollController;
        #ifdef rollInnerController
        rollInnerControllerType pid_vroll = rollInnerController;
        #endif
    #endif

    #ifdef useYawController
        yawControllerType pid_yaw = yawController;
        #ifdef yawInnerController
        yawInnerControllerType pid_vyaw = yawInnerController;
        #endif
    #endif
};

pryt stabilize(pryt setpoint, allMpu data,conStruct *cons, float hoverthrust){
    pryt output;

    #ifdef usePitchController
        output.pitch  = cons->pid_pitch.update(setpoint.pitch-data.ypr[2]);
        #ifdef pitchInnerController
            output.pitch = cons->pid_vpitch.update(output.pitch-(data.gyro.x/GYROCOMP));
        #endif
    #endif

    #ifdef useRollController
        output.roll = cons->pid_roll.update(setpoint.roll-data.ypr[1]);
        #ifdef rollInnerController
            output.roll = cons->pid_vroll.update(output.roll-(data.gyro.y/GYROCOMP));
        #endif
    #endif

    #ifdef useYawController
        output.yaw = cons->pid_yaw.update(setpoint.yaw-data.ypr[0]);
        #ifdef yawInnerController
            output.yaw = cons->pid_vyaw.update(output.yaw-(data.gyro.z/GYROCOMP));
        #endif
    #endif

    #ifdef useThrustController
        output.thrust = cons->pid_thrust.update(setpoint.thrust) + hoverthrust;
        #ifdef thrustInnerController
            output.thrust = cons->pid_vthrust.update(output.thrust - data.aaReal.z);
        #endif
    #endif

    return output;
}
