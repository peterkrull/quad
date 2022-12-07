#pragma once

#include <Arduino.h>
#include "DShotRMT.h"
#include "module/dataStructures.h"

struct motor {
    uint16_t speed = DSHOT_THROTTLE_MIN;
    DShotRMT dshot;
    bool reversed;
    bool rmt_init = false;
    uint8_t rmt_channel;
    uint8_t gpio_pin;
    bool standby = true;
    bool ready = false;
};

struct motors4 {
    motor m1;
    motor m2;
    motor m3;
    motor m4;
};

/*
Sets the speeds of 4 motors simultaniously. Each motors speed value has to be
defined using the `motors4->m1.speed` variable for each of the four motors.
*/
void sendSpeeds(motors4 * motors) {
    motors->m1.dshot.sendThrottle(motors->m1.speed);
    motors->m2.dshot.sendThrottle(motors->m2.speed);
    motors->m3.dshot.sendThrottle(motors->m3.speed);
    motors->m4.dshot.sendThrottle(motors->m4.speed);
}

void setSpeeds(motors4 * motors, uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4) {
    motors->m1.speed = s1;
    motors->m2.speed = s2;
    motors->m3.speed = s3;
    motors->m4.speed = s4;
}


void setSpeedsSame(motors4 * motors, uint16_t speed) {
    motors->m1.speed = speed;
    motors->m2.speed = speed;
    motors->m3.speed = speed;
    motors->m4.speed = speed;
}

/*
m1 => thrust + pitch - roll - yaw
m2 => thrust - pitch + roll - yaw
m3 => thrust + pitch + roll + yaw
m4 => thrust - pitch - roll + yaw
*/
void motorMixing(motors4 * motor, float thrust, float pitch, float roll, float  yaw){
    // It is important to constrain the floating point value before assigning 
    // it to the output uint16_t variables to prevent overflow.
    motor -> m1.speed = (uint16_t)constrain(thrust + pitch - roll - yaw,DSHOT_THROTTLE_MIN+20,DSHOT_THROTTLE_MAX);
    motor -> m2.speed = (uint16_t)constrain(thrust - pitch + roll - yaw,DSHOT_THROTTLE_MIN+20,DSHOT_THROTTLE_MAX);
    motor -> m3.speed = (uint16_t)constrain(thrust + pitch + roll + yaw,DSHOT_THROTTLE_MIN+20,DSHOT_THROTTLE_MAX);
    motor -> m4.speed = (uint16_t)constrain(thrust - pitch - roll + yaw,DSHOT_THROTTLE_MIN+20,DSHOT_THROTTLE_MAX);
}

void motorInit(void *pvParameters) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
    ((motor*)pvParameters)->rmt_init = !((motor*)pvParameters)->dshot.install(
        gpio_num_t(((motor*)pvParameters)->gpio_pin),
        rmt_channel_t(((motor*)pvParameters)->rmt_channel)
        );
    if (((motor*)pvParameters)->rmt_init){         
        ((motor*)pvParameters)->dshot.init(true);
        ((motor*)pvParameters)->dshot.setReversed(((motor*)pvParameters)->reversed);
        const TickType_t xFrequency = 100;
        TickType_t xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, int(1000 / xFrequency));
        while (((motor*)pvParameters)->standby) {
            uint16_t speed = ((motor*)pvParameters)->speed;
            if (speed < 48) speed = 48; if (speed > 2047) speed = 2047;
            ((motor*)pvParameters)->ready = true;
            ((motor*)pvParameters)->dshot.sendThrottle(speed);
            vTaskDelayUntil(&xLastWakeTime, int(1000 / xFrequency));
        }
    } vTaskDelete( NULL );
}

class dshot_motor {

    public:

        dshot_motor(uint8_t gpio_pin, uint8_t rmt_channel, bool reversed){
            m.gpio_pin = gpio_pin;
            m.rmt_channel = rmt_channel;
            m.reversed = reversed;
        }

        void startTaskMotorInit(){
            xTaskCreate(motorInit, "motorTask", 5000, &m, 1, NULL);
        }

        // Take throttle as  [ 0.0 .. 1.0 ]
        esp_err_t sendMotorThrottle(float throttle) {
            m.standby = false;
            throttle = constrain( mapf(throttle,0.0,1.0,48.0,2047.0) , 48 , 2047 );
            if ((bool*)motors_enable){
                return m.dshot.sendThrottle(throttle);
            } else {
                return m.dshot.sendThrottle(48);
            }
        }

        void setMotorEnablePtr(bool * motors_enable_ptr){
            motors_enable = motors_enable_ptr;
        }

        bool isReady(){ return m.ready; }


    private:
        motor m;
        bool * motors_enable = NULL;
};
