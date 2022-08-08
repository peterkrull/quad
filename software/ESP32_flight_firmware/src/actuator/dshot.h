#include <Arduino.h>
#include "DShotRMT.h"

#ifndef dshot_h
#define dshot_h

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

double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
    return ((x - in_min) * (out_max - out_min) + ((in_max - in_min) / 2)) / (in_max - in_min) + out_min;
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

#endif