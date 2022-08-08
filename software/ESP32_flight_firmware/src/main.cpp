#include <Arduino.h>
#include "freertos/FreeRTOS.h"

#include "configuration.h"
#include "board/definitions.h"
#include "health/power.h"
#include "sensor/imu/imu.h"
#include "sensor/gps/gps.h"
#include "com/wifi.h"
#include "actuator/dshot.h"
#include "sigproc/kalman.h"

// wifi_creds credentials = wifi_creds();
// AsyncUDP udp;
// const int mm_udp_port = 51000;
// const char *udp_addr = "255.255.255.255";

#define MAIN_LOOP_FREQ 500

IMU imu;
GPS gps;

Kalman kalman;

dshot_motor M[] = {
    dshot_motor(M1_pin,0,REVERSE_MOTOR_M1),
    dshot_motor(M2_pin,1,REVERSE_MOTOR_M2),
    dshot_motor(M3_pin,2,REVERSE_MOTOR_M3),
    dshot_motor(M4_pin,3,REVERSE_MOTOR_M4),
};

#ifdef VBAT_pin
Battery bat = Battery(BATTERY_VMIN,BATTERY_VMAX,VBAT_pin);
#endif

void setup(){

    bool motors_enable = true;

    for (int i = 0 ; i < 4 ; i++) {
        M[i].setMotorEnablePtr(&motors_enable);
        M[i].startTaskMotorInit();
    } delay(50);

    // Setup USB serial connection
    #if ENABLE_SERIAL_PRINT
    Serial.begin(BAUDRATE);
    #endif

    // Initialize battery monitoring daemon
    #ifdef VBAT_pin
    while ( bat.isLow() ) { Serial.printf("Voltage too low, aborting!\nVoltage : %f V\n",bat.getVoltage()); sleep(1); } 
    #endif

    imu.initialize();
    gps.initialize(&Serial1,GPS_RX,GPS_TX);

}





void loop(){
    uint32_t prev_time = micros();
    while (true) {
        // ensure main loop is running at MAIN_LOOP_FREQ Hz
        if (micros() >= prev_time + (1e6/MAIN_LOOP_FREQ)) {
            prev_time += (1e6/MAIN_LOOP_FREQ);

            String time,acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z,lat,lng,alt,vel;

            // if (imu.mag.isUpdated()) {
            //     kalman.newMag(imu.mag.getMag(),imu.mag.getTimestamp());
            // }

            // if (baro.isUpdated()) {
            //     kalman.newBaroAltitude(baro.getAltitude(),baro.getTimestamp());
            // }

            if (imu.isUpdated()) {
                kalman.newAccel( imu.getAccel() , imu.getTimestamp() );
                kalman.newGyro( imu.getGyro() , imu.getTimestamp() );

                #ifdef HAS_MAGNETOMETER
                kalman.newMag(imu.mag.getMag(),imu.mag.getTimestamp());
                #endif
                    
                { // printer
                    char buffer [6];

                    time.concat(imu.getTimestamp());
                    
                    sprintf(buffer,"%f",imu.getAccel().x);
                    acc_x.concat(buffer);

                    sprintf(buffer,"%f",imu.getAccel().y);
                    acc_y.concat(buffer);

                    sprintf(buffer,"%f",imu.getAccel().z);
                    acc_z.concat(buffer);

                    sprintf(buffer,"%f",imu.getGyro().x);
                    gyr_x.concat(buffer);

                    sprintf(buffer,"%f",imu.getGyro().y);
                    gyr_y.concat(buffer);

                    sprintf(buffer,"%f",imu.getGyro().z);
                    gyr_z.concat(buffer);
                }
            }

            if (gps.isUpdated()) {
                kalman.newGpsLocation( gps.getLatitude(), gps.getLongitude() , gps.getTimestamp() );
                kalman.newGpsAltitude( gps.getAltitude(), gps.getTimestamp() );
                kalman.newGpsVelocity( gps.getVelocity(), gps.getTimestamp() );

                { // printer
                    char buffer [6];

                    sprintf(buffer,"%f",gps.getLatitude());
                    lat.concat(buffer);

                    sprintf(buffer,"%f",gps.getLongitude());
                    lng.concat(buffer);

                    sprintf(buffer,"%f",gps.getAltitude());
                    alt.concat(buffer);

                    sprintf(buffer,"%f",gps.getVelocity());
                    vel.concat(buffer);
                }
            }
        
            if ( M[0].isReady() && M[1].isReady() && M[2].isReady() && M[3].isReady() ) {

                M[0].sendMotorThrottle(0.05);
                M[1].sendMotorThrottle(0.05);
                M[2].sendMotorThrottle(0.05);
                M[3].sendMotorThrottle(0.05);

            }

            Serial.printf("%u, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n",millis(),acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z,lat,lng,alt,vel);
        }

        kalman.update();

    }
}