#include <Arduino.h>
#include <FreeRTOS.h>
#include "DShotRMT.h"
#include "wifi_credentials.h"
#include "sigProc.h"
#include <math.h>
#include <WiFi.h>
#include "MPU6050_6Axis_MotionApps612.h"
#include <AsyncUDP.h>
#include "TinyGPSPlus.h"
#include "setupGPS.h"
#include "configuration.h"
#include "dataStructures.h"
#include "boards.h"
#include "controllers.h"

wifi_creds credentials = wifi_creds();
AsyncUDP udp;
const int mm_udp_port = 51000;
const char *udp_addr = "255.255.255.255";

// MPU / IMU 
MPU6050 mpu;
allMpu mpuData;

// GPS SETUP
TinyGPSPlus gps;

// CONTROLLERS
conStruct controllers;

bool motors_enable = false;
uint16_t hoverthrust = hoverThrust;
int16_t offset = 0;

#define udpTimeoutMS 500

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

void vKeepConnection(void *pvParameters) {
    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            digitalWrite(LED_BUILTIN,LOW);
            WiFi.begin(credentials.SSID, credentials.password);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        } else {
            digitalWrite(LED_BUILTIN,HIGH);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void vTelemetry(void *pvParameters) {

    if (WiFi.status() == WL_CONNECTED)
    {
        //udp.begin(udp_port);
    }

    const TickType_t xFrequency = 10;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            vTaskDelayUntil(&xLastWakeTime, int(1000 / xFrequency));
            // uint32_t time_ms = millis();
            // digitalWrite(LED_BUILTIN, HIGH);
            // udp.beginPacket(udp_addr, udp_port);
            // udp.write((const uint8_t *)&time_ms, sizeof(time_ms));
            // udp.write((const uint8_t *)&data_imu, sizeof(data_imu));
            // udp.write((const uint8_t *)&gps_data, sizeof(gps_data));
            // udp.endPacket();
            // digitalWrite(LED_BUILTIN, LOW);
        }
    }
}

void parseUDP(AsyncUDPPacket &packet){
    
    // Heartbeat time out detections
    /* THIS IS FUNDAMENTALLY FLAWED!
    
    If the UDP stream suddenly breaks up, this function
    is never used and thus the timeout wont happen.

    The correct solution would be to simply pass latest time
    stamp to a sanityCheck function, that occasionally checks
    for strange behaviour.

    uint32_t xmillis = millis();
    
    if (udpTimeoutCntr == 0){
        udpTimeoutCntr = xmillis;
    } else {
        uint16_t UDPdeltaTime = xmillis - udpTimeoutCntr;
        udpTimeoutCntr = xmillis;
    } if (UDPdeltaTime > udpTimeoutMS) {motors_enable = 0;}
    
    */
    String instr = packet.readStringUntil(*"\n");

    // Set motor enable or disable
    int m_index = instr.indexOf("m");
    if (m_index>-1){
        motors_enable = instr.substring(m_index+1,m_index+2).toInt();Serial.printf("m%d\n",motors_enable);
        if (motors_enable != 1 || motors_enable != 0) { motors_enable = 0;}
    }

    // Set pitch angle
    int p_index = instr.indexOf("p");
    if (p_index>-1){
        pryt_sp.pitch = instr.substring(p_index+1,instr.length()).toFloat();Serial.printf("p%f\n",pryt_sp.pitch);
        if (pryt_sp.pitch > 10 || pryt_sp.pitch < 10) { pryt_sp.pitch = 0;}
    }

    // Set hover thrust
    int h_index = instr.indexOf("h");
    if (h_index>-1){
        hoverthrust = instr.substring(h_index+1,instr.length()).toInt();Serial.printf("h%d\n",hoverthrust);
        if (hoverthrust > 1500 || hoverthrust < 0) { hoverthrust = 0;}
    }

    // Set hover thrust
    int o_index = instr.indexOf("o");
    if (o_index>-1){
        offset = instr.substring(o_index+1,instr.length()).toInt();Serial.printf("o%d\n",offset);
        if (offset > 5000 || offset < 5000) { offset = 0;}
    }
}

void vUDPlistener(void *pvParameters) {
    const TickType_t xFrequency = 10;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, int(1000 / xFrequency));
        if (WiFi.status() == WL_CONNECTED)
        {
            if (udp.listen(mm_udp_port)){
                udp.onPacket(parseUDP);
            }
        }
    }
}

void vTinyGPSpull(void *pvParameters){
    while(true){
        vTaskDelay(10 / portTICK_PERIOD_MS);
        //uint32_t start_time = micros();
        while (Serial1.available() > 0){
            if (((TinyGPSPlus*)pvParameters)->encode(Serial1.read())){
                digitalWrite(LED_BUILTIN,((TinyGPSPlus*)pvParameters)->location.isValid());
            }
        }
    }
}

esp_err_t sendMotorThrottle(motor *M, uint16_t speed) {
    if (speed < 48) speed = 48; if (speed > 2047) speed = 2047;
    if (motors_enable){
        return ((motor*)M)->dshot.sendThrottle(speed);
    } else {
        return ((motor*)M)->dshot.sendThrottle(48);
    }
}

void getAllMPU(allMpu *data){
    mpu.dmpGetQuaternion(&data->q, data->fifoBuffer);
    mpu.dmpGetAccel(&data->aa, data->fifoBuffer);
    mpu.dmpGetGravity(&data->gravity, &data->q);
    mpu.dmpGetLinearAccel(&data->aaReal, &data->aa, &data->gravity);
    mpu.dmpGetYawPitchRoll(data->ypr, &data->q, &data->gravity);
    mpu.dmpGetGyro(&data->gyro, data->fifoBuffer);
}

void motorMixing(quadcopter_motors *motor, pryt pryt){
    motor -> M1 = pryt.thrust + pryt.pitch - pryt.roll - pryt.yaw;
    motor -> M2 = pryt.thrust - pryt.pitch + pryt.roll - pryt.yaw;
    motor -> M3 = pryt.thrust + pryt.pitch + pryt.roll + pryt.yaw;
    motor -> M4 = pryt.thrust - pryt.pitch - pryt.roll + pryt.yaw;
}

low_pass totalmotorsanity_lp = low_pass(0.5);
low_pass M1sanity_lp = low_pass(0.3);
low_pass M2sanity_lp = low_pass(0.3);
low_pass M3sanity_lp = low_pass(0.3);
low_pass M4sanity_lp = low_pass(0.3);
low_pass AAsanity_lp = low_pass(0.1);

uint8_t sanityCheck(){

    motors_enable = 0;

    // Check for 90 deg pitch or roll
    if (abs(mpuData.ypr[1]) > PI/2 || abs(mpuData.ypr[2]) > PI/2) {return 0;}

    // Check for prolonged total motor power
    if (totalmotorsanity_lp.update(motors.M1+motors.M2+motors.M3+motors.M4)/4 > 1000) {return 0;}

    // Check for proloneged individual motor power
    if (M1sanity_lp.update(motors.M1) > 1500) {return 0;}
    if (M2sanity_lp.update(motors.M2) > 1500) {return 0;}
    if (M3sanity_lp.update(motors.M3) > 1500) {return 0;}
    if (M3sanity_lp.update(motors.M4) > 1500) {return 0;}

    // Check for abnormal acceleration values
    if (AAsanity_lp.update(mpuData.aaReal.getMagnitude()) > 5000) {return 0;}

    motors_enable = 1;

    return 1;
}

void vHyperLoop(void *pvParameters){

    while (true) {
        if (mpu.dmpGetCurrentFIFOPacket(mpuData.fifoBuffer)) {
            
            getAllMPU(&mpuData);

            pryt PRYT = stabilize(pryt_sp,mpuData,&controllers,hoverthrust);

            motorMixing(&motors,PRYT);

            if (sanityCheck()){
                sendMotorThrottle(&M1,motors.M1);
                sendMotorThrottle(&M2,motors.M2);
                sendMotorThrottle(&M3,motors.M3);
                sendMotorThrottle(&M4,motors.M4);
            }

            // This should be interrupt-based
            vTaskDelay(2 / portTICK_PERIOD_MS);
        }
    }
}

void vMotorInit(void *pvParameters){
    const TickType_t xFrequency = 10;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, int(1000 / xFrequency));
        if (M1.ready && M2.ready && M3.ready && M4.ready){
            M1.standby = false; M2.standby = false; M3.standby = false; M4.standby = false;
            xTaskCreate(vHyperLoop,"HyperLoop",3000,NULL,1,NULL);
            vTaskDelete( NULL );
        }
    } 
}

//
// Setup and loop
//

void setup() {

    // Initialize LEDs and set high
    pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN,HIGH);

    // Initialize serial (USB)
    Serial.begin(115200);
    while (!Serial){
        delay(10);
    } Serial.println("Serial initialized");

    // Initialize MPU
    Wire.begin();
    Serial.println("Wire Initialized");
    delay(10);

    // Set motor configurations
    M1.reversed = !reverseMotorDir; M1.gpio_pin = M1_pin;   M1.rmt_channel = 0;
    M2.reversed = !reverseMotorDir; M2.gpio_pin = M2_pin;   M2.rmt_channel = 1;
    M3.reversed = reverseMotorDir;  M3.gpio_pin = M3_pin;   M3.rmt_channel = 2;
    M4.reversed = reverseMotorDir;  M4.gpio_pin = M4_pin;   M4.rmt_channel = 3;
    
    // Create motor initialization tasks
    xTaskCreate(motorInit, "motorGovM1", 5000, &M1, 1, NULL);
    xTaskCreate(motorInit, "motorGovM2", 5000, &M2, 1, NULL);
    xTaskCreate(motorInit, "motorGovM3", 5000, &M3, 1, NULL);
    xTaskCreate(motorInit, "motorGovM4", 5000, &M4, 1, NULL);

    mpu.initialize();
    Serial.println("MPU Initialized");
    mpuData.devStatus = mpu.dmpInitialize();
    if (mpuData.devStatus == 0){
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        mpu.setDLPFMode(MPU6050_DLPF_BW_256);
        mpuData.packetSize = mpu.dmpGetFIFOPacketSize();
    } Serial.println("devStatus received");

    // Initialize GPS
    GPSSerialInit(Serial1,GPS_RX,GPS_TX);
    Serial.println("GPS initializd");

    // Catch unsuccesful initializations
    if (mpuData.devStatus == 0) {

        // Create initial system tasks
        xTaskCreate(vKeepConnection, "KeepConnection", 3000, NULL, 4, NULL);
        xTaskCreate(vUDPlistener, "Commander", 2000, NULL, 3, NULL);
        xTaskCreate(vTinyGPSpull, "tinyGPSpull", 2000, &gps, 3, NULL);
        xTaskCreate(vMotorInit,"MotorInit",1000,NULL,2,NULL);

        // Make audible sound when setup is complete
        M1.dshot.beep(); M2.dshot.beep(); M3.dshot.beep(); M4.dshot.beep();

        Serial.println("SETUP COMPLETE");
    } else {
        Serial.println("SETUP FAILED");
        while (true) {
            digitalWrite(LED_BUILTIN,HIGH);
            delay(250);
            digitalWrite(LED_BUILTIN,LOW);
            delay(250);
        }
    }
}

void loop() { vTaskDelay(10000 / portTICK_PERIOD_MS); }
