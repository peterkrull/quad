#include "includes.h"

// wifi_creds credentials = wifi_creds();
// AsyncUDP udp;
// const int mm_udp_port = 51000;
// const char *udp_addr = "255.255.255.255";

#define MAIN_LOOP_FREQ 500
#define MOTORS_ENABLED true
#define BLOCK_ALL_MOTORS false
#define SBUS_ENABLE true

#if SBUS_ENABLE
bfs::SbusRx sbus = bfs::SbusRx(&Serial2,16,17,true);
bfs::SbusData sbus_data;
#endif

IMU imu;
GPS gps;

Kalman kalman3d;
Kalman kalmanAlt;

motors4 motors;

// CONTROLLERS
PID pid_pitch, pid_roll, pid_yaw, pid_thrust, pid_vpitch, pid_vroll, pid_vyaw, pid_vthrust;

#define circular_constraint(x,min,max) ( x < min ) ? ( x + max-min ) : ( x > max ) ? ( x + min-max ) : x

void setup(){
    #if SBUS_ENABLE
    sbus.Begin();
    #endif

    pinMode(LED_RED,OUTPUT);

    // Set motor configurations and start intial runner
    motors.m1.reversed = M1_REVERSE_DIR;    motors.m1.gpio_pin = M1_pin;    motors.m1.rmt_channel = M1_RMT_CH;  xTaskCreate(motorInit, "motorGovM1", 5000, &motors.m1, 1, NULL);
    motors.m2.reversed = M2_REVERSE_DIR;    motors.m2.gpio_pin = M2_pin;    motors.m2.rmt_channel = M2_RMT_CH;  xTaskCreate(motorInit, "motorGovM2", 5000, &motors.m2, 1, NULL);
    motors.m3.reversed = M3_REVERSE_DIR;    motors.m3.gpio_pin = M3_pin;    motors.m3.rmt_channel = M3_RMT_CH;  xTaskCreate(motorInit, "motorGovM3", 5000, &motors.m3, 1, NULL);
    motors.m4.reversed = M4_REVERSE_DIR;    motors.m4.gpio_pin = M4_pin;    motors.m4.rmt_channel = M4_RMT_CH;  xTaskCreate(motorInit, "motorGovM4", 5000, &motors.m4, 1, NULL);

    // Setup USB serial connection
    #if ENABLE_SERIAL_PRINT
    Serial.begin(BAUDRATE);
    #endif

    // Initialize sensors
    bool initialized = true;
    initialized &= imu.initialize();
    initialized &= gps.initialize(&Serial1,GPS_RX,GPS_TX);

    if (initialized) {

        // Absolute controllers             Velocity controllers
        pid_thrust = CON_THRUST_OUT_c;      pid_vthrust = CON_THRUST_INN_c;
        pid_pitch = CON_PITCH_OUT_c;        pid_vpitch = CON_PITCH_INN_c;
        pid_pitch.setOutputLimit(-40,40);   pid_vpitch.setOutputLimit(-400,400);
        pid_roll = CON_ROLL_OUT_c;          pid_vroll = CON_ROLL_INN_c;
        pid_roll.setOutputLimit(-40,40);    pid_vroll.setOutputLimit(-400,400);
        pid_yaw = CON_YAW_OUT_c;            pid_vyaw = CON_YAW_INN_c;

    } else {
        // Initialization failed
        while (true) { Serial.println("Initialization failed!"); delay(1000); }
    }
}

struct ledBlinker {
    int period;
    int led_pin;
    bool enable;
};

void blink(void *pvParameters) {
    while (true) {
        if ( ((ledBlinker*)pvParameters)->enable ){
            digitalWrite(((ledBlinker*)pvParameters)->led_pin,!digitalRead(((ledBlinker*)pvParameters)->led_pin));
            vTaskDelay( ((ledBlinker*)pvParameters)->period / portTICK_PERIOD_MS);
        } else {
            digitalWrite(((ledBlinker*)pvParameters)->led_pin,LOW);
            vTaskDelay( 50 / portTICK_PERIOD_MS);
        }
    }
}


void loop(){

    tri_switch E_switch = tri_switch::idle;
    tri_switch B_switch = tri_switch::idle; // Arm motors 
    tri_switch C_switch = tri_switch::idle; // Run motors
    tri_switch F_switch = tri_switch::idle;

    float thrust_control = 0.;
    float pitch_control = 0.;
    float roll_control = 0.;
    float yaw_control = 0.;

    float yaw_sp = 0.0;

    uint32_t prev_time = micros();

    // Leave standby mode of ESCs
    motors.m1.standby = false;
    motors.m2.standby = false;
    motors.m3.standby = false;
    motors.m4.standby = false;

    uint8_t arm_sequence = 0;

    ledBlinker red_led;
    red_led.period = 300;
    red_led.led_pin = LED_RED;
    red_led.enable = true;

    uint32_t sbus_time = millis();

    xTaskCreate(blink, "red_led_blinker", 1000, &red_led, 5, NULL);

    while (true) {

        #if SBUS_ENABLE

        // Fetch controller data
        if( sbus.Read() ) { 
            sbus_data = sbus.data();

            thrust_control = sbus_range(sbus_data.ch[2]);
            pitch_control = sbus_range(sbus_data.ch[1]);
            roll_control = sbus_range(sbus_data.ch[0]);
            yaw_control = sbus_range(sbus_data.ch[3]);

            E_switch = sbus_switch(sbus_data.ch[4]);
            B_switch = sbus_switch(sbus_data.ch[5]);
            C_switch = sbus_switch(sbus_data.ch[6]);
            F_switch = sbus_switch(sbus_data.ch[7]);
            if (!sbus_data.failsafe) { 
                sbus_time = millis();
            }
        }

        // Act on activation switch
        if ( C_switch == tri_switch::idle ) {
            setSpeedsSame(&motors, DSHOT_THROTTLE_MIN);
        } else if ( C_switch == tri_switch::middle ) {
            setSpeedsSame(&motors, DSHOT_THROTTLE_MIN + 20);
        }

        // Arming sequence
        if (arm_sequence < 3 && sbus_time + 1000 >= millis()) {
            if (arm_sequence == 2 && C_switch == tri_switch::active && thrust_control < -0.9) {arm_sequence = 3; red_led.enable = true; red_led.period = 50;}
            if (arm_sequence == 1 && C_switch == tri_switch::middle && thrust_control < -0.9) {arm_sequence = 2; red_led.enable = true; red_led.period = 500;}
            if (arm_sequence == 0 && C_switch == tri_switch::idle ) {arm_sequence = 1; red_led.enable = false;}
        } else if ( sbus_time + 1000 < millis() ) {
            arm_sequence = 0; red_led.enable = false; setSpeedsSame(&motors, DSHOT_THROTTLE_MIN);
        }

        #endif

        #if BLOCK_ALL_MOTORS
        setSpeedsSame(&motors, DSHOT_THROTTLE_MIN);
        #endif

        if (arm_sequence == 3) { sendSpeeds(&motors); }
        else { setSpeedsSame(&motors, DSHOT_THROTTLE_MIN); sendSpeeds(&motors); }

        // ensure main loop is running at MAIN_LOOP_FREQ Hz
        if (micros() >= prev_time + (1e6/MAIN_LOOP_FREQ)) {
            prev_time += (1e6/MAIN_LOOP_FREQ);

            if (imu.isUpdated()) {
                Quaternion quat = imu.getQuaternion();
                VectorFloat acc = imu.getAccel().getRotated(&quat);
                VectorFloat gyro = imu.getGyro();
                VectorFloat pry = imu.getPry();

                yaw_sp = (C_switch != tri_switch::active) ? pry.z : ( abs(yaw_control) > 0.01 ) ? yaw_sp + yaw_control*0.01 : yaw_sp;

                yaw_sp = circular_constraint(yaw_sp,-PI,PI);

                kalman3d.newAcc(acc,micros());

                float con_pitch = pid_pitch.update(pitch_control-pry.y);
                float con_vpitch = pid_vpitch.update(con_pitch-gyro.x);

                float con_roll = pid_roll.update(-roll_control+pry.x);
                float con_vroll = pid_vroll.update(con_roll-gyro.y);

                float con_yaw = pid_yaw.update(yaw_sp-pry.z,-PI,PI);
                float con_vyaw = pid_vyaw.update(con_yaw+gyro.z);

                float thrust = mapf(thrust_control,-1,1,DSHOT_THROTTLE_MIN , 900);

                // Compensate for lost vertical thrust when pitching and rolling
                thrust /= cos((abs(pry.y) < PI/4 )?pry.y:PI/4)*cos((abs(pry.x) < PI/4 )?pry.x:PI/4);
                
                motorMixing(&motors,thrust,con_vpitch,con_vroll,con_vyaw);

                }

            if (gps.isUpdated()) {
                double lat = gps.getLatitude();
                double lon = gps.getLongitude();
                double alt = gps.getAltitude();
                double vel = gps.getVelocity();

                lat = lat+lon+alt+vel;
            };

        //     String time,acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z,lat,lng,alt,vel;

        //     if (baro.isUpdated()) {
        //         kalmanAlt.newBaroAltitude(baro.getAltitude(),baro.getTimestamp());
        //     }

        //     if () {
        //         kalman3d.newLidarAltitude(baro)
        //     }

        // kalman3d.predict(1.0/(float)MAIN_LOOP_FREQ);
        }
        vTaskDelay( 1 / portTICK_PERIOD_MS);
    }
}


// loop {
//  control
//  measure
//  evaluate
//  actuate
// }