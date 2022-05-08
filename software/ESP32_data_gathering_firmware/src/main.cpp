#include <Arduino.h>
#include <FreeRTOS.h>
#include "DShotRMT.h"
#include "wifi_credentials.h"

#include <math.h>
#include <WiFi.h>

#include <AsyncUDP.h>

#include <WiFiUdp.h>

#include "TinyGPSPlus.h"
#include "sensor/gps/setupGPS.h"

#include "configuration.h"

#include "sigproc/sigProc.h"
#include "module/dataStructures.h"
#include "board/definitions.h"
#include "control/controllers.h"
#include "health/sanityCheck.h"

wifi_creds credentials = wifi_creds();
AsyncUDP udp;
const int mm_udp_port = 51000;
const char *udp_addr = "255.255.255.255";

WiFiUDP udp_out;
const int udp_port_2 = 50000;
const char *udp_addr_2 = "192.168.1.193";

quadcopter_motors motors;
pryt pryt_sp;
motor M1,M2,M3,M4;

// MPU / IMU 
MPU6050 mpu;
allMpu mpuData;

// GPS SETUP
TinyGPSPlus gps;

// CONTROLLERS
conStruct controllers;

sanityCheck sanity;

bool motors_enable = false;
uint16_t hoverthrust = HOVER_THRUST;
int16_t offset = 0;

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

bool udp_out_init = false;

void udpPrint(String message){
    if (WiFi.status() == WL_CONNECTED)
    {
        if (!udp_out_init) {
            udp_out.begin(udp_port_2);
            udp_out_init = true;
        } else {
            udp_out.beginPacket(udp_addr_2, udp_port_2);
            udp_out.print(message);
            udp_out.endPacket();
        }
    }
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
            // digitalWrite(LED_BUILTIN, LOW)
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

    // Serial.println("GOT: "+instr);

    // Set motor enable or disable
    int m_index = instr.indexOf("m");
    if (m_index>-1){
        motors_enable = instr.substring(m_index+1,m_index+2).toInt();Serial.printf("m%d\n",motors_enable);
        if (!(motors_enable == 1 || motors_enable == 0)) { motors_enable = 0;}
    }

    // Set pitch angle
    int p_index = instr.indexOf("p");
    if (p_index>-1){
        pryt_sp.pitch = instr.substring(p_index+1,instr.length()).toFloat();Serial.printf("p%f\n",pryt_sp.pitch);
        if (pryt_sp.pitch > 10 || pryt_sp.pitch < -10) { pryt_sp.pitch = 0;}
    }

    // Set hover thrust
    int h_index = instr.indexOf("h");
    if (h_index>-1){
        hoverthrust = instr.substring(h_index+1,instr.length()).toInt();Serial.printf("h%d\n",hoverthrust);
        if (hoverthrust > 1500 || hoverthrust < 0) { hoverthrust = 0;}
    }

    // Set offset angle
    int o_index = instr.indexOf("o");
    if (o_index>-1){
        offset = instr.substring(o_index+1,instr.length()).toInt();Serial.printf("o%d\n",offset);
        if (offset > 5000 || offset < 5000) { offset = 0;}
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        if (!udp_out_init) {
            udp_out.begin(udp_port_2);
            udp_out_init = true;
        } else if (instr.length() > 0){
            udp_out.beginPacket(udp_addr_2, udp_port_2);
            udp_out.print(instr);
            udp_out.endPacket();
        }
    }
}

uint32_t udpTime = 0;
bool udpTimeout = false;

void vUDPlistener(void *pvParameters) {
    const TickType_t xFrequency = 10;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, int(1000 / xFrequency));
        if (WiFi.status() == WL_CONNECTED)
        {
            if (udp.listen(mm_udp_port)){
                udpTime = millis();
                udp.onPacket(parseUDP);
            }

            // Handle timeouts by checking 
            if (udpTime == 0) {
                udpTime = millis();
            } else if (millis() - udpTime > UDP_TIMEOUT_MS) {
                udpTimeout = true;
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

void vGetAllMPU(void *pvParameters){
    while (true) {
        if (mpu.dmpGetCurrentFIFOPacket(mpuData.fifoBuffer)) {
            getAllMPU((allMpu*)pvParameters);
            vTaskDelay(2 / portTICK_PERIOD_MS);
        }
    }
}

void motorMixing(quadcopter_motors *motor, pryt pryt){
    motor -> M1 = (uint16_t) limiter( pryt.thrust + pryt.pitch - pryt.roll - pryt.yaw, 48, 2047);
    motor -> M2 = (uint16_t) limiter( pryt.thrust - pryt.pitch + pryt.roll - pryt.yaw, 48, 2047);
    motor -> M3 = (uint16_t) limiter( pryt.thrust + pryt.pitch + pryt.roll + pryt.yaw, 48, 2047);
    motor -> M4 = (uint16_t) limiter( pryt.thrust - pryt.pitch - pryt.roll + pryt.yaw, 48, 2047);
}

PID pid_pitch = PID(5,0,0.15,0,true);
PID pid_vpitch = PID(20,0.333,0.05,0.01,true);
pryt PRYT;

uint32_t counter = 0;

void vHyperLoop(void *pvParameters){

    while (true) {
        if (mpu.dmpGetCurrentFIFOPacket(mpuData.fifoBuffer)) {
            
            getAllMPU(&mpuData);

            // pryt PRYT = stabilize(pryt_sp,mpuData,&controllers,hoverthrust);
            
            float out_pitch  = pid_pitch.update(pryt_sp.pitch-mpuData.ypr[2]);
            PRYT.pitch = pid_vpitch.update(out_pitch-((float)mpuData.gyro.x/GYROCOMP));

            counter ++ ;
            if (counter >= 5) {
                char buffer[40];
                sprintf(buffer,"%.2f ; %.2f ; %.2f ; %.2f ; %.2f",PRYT.pitch,PRYT.thrust,pryt_sp.pitch,mpuData.ypr[2],((float)mpuData.gyro.x/GYROCOMP));
                udpPrint(buffer);
                counter = 0;
            }

            PRYT.thrust = hoverthrust;
            motorMixing(&motors,PRYT);

            // if (sanity.check(motors,mpuData)){
            if (!udpTimeout){
                // udpPrint(String(motors.M1)+","+String(motors.M2)+","+String(motors.M3)+","+String(motors.M4)+","+String(mpuData.ypr[]));
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

void vPrinter(void *pvParameters){
    const TickType_t xFrequency = 100;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        Serial.printf("%f , %f , %i , %i , %i , %i , %i , %i\n",gps.location.lat(),gps.location.lng(),mpuData.aaReal.x,mpuData.aaReal.y,mpuData.aaReal.z,mpuData.gyro.x,mpuData.gyro.y,mpuData.gyro.z);
        vTaskDelayUntil(&xLastWakeTime, int(1000 / xFrequency));
    }
}

void setup() {

    pinMode(LED_RED, OUTPUT); digitalWrite(LED_RED,HIGH);

    // Initialize serial (USB)
    Serial.begin(115200);
    while (!Serial){
        delay(10);
    } Serial.println("Serial initialized");

    // Initialize MPU
    Wire.begin();
    Serial.println("Wire Initialized");
    delay(10);

    mpu.initialize();
    Serial.println("MPU Initialized");
    mpuData.devStatus = mpu.dmpInitialize();
    Serial.println("\ndevStatus received");
    if (mpuData.devStatus == 0){
        Serial.println("Devstatus GOOD");
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        mpu.setDLPFMode(MPU6050_DLPF_BW_188);
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
        mpuData.packetSize = mpu.dmpGetFIFOPacketSize();
    } else {Serial.println("Devstatus FAIL");}

    // Initialize GPS
    GPSSerialInit(Serial1,GPS_RX,GPS_TX);
    Serial.println("GPS initialized");

    // Catch unsuccesful initializations
    if (mpuData.devStatus == 0) {

        xTaskCreate(vTinyGPSpull, "tinyGPSpull", 2000, &gps, 2, NULL);
        xTaskCreate(vGetAllMPU, "getAllMPU", 5000, &mpuData, 1, NULL);
        xTaskCreate(vPrinter,"printer",2000,NULL,2,NULL);

        Serial.println("SETUP COMPLETE");
        digitalWrite(LED_RED,LOW);
    } else {
        Serial.println("SETUP FAILED");
        while (true) {
            digitalWrite(LED_RED,HIGH);
            delay(250);
            digitalWrite(LED_RED,LOW);
            delay(250);

            udpPrint("Drone could not initialize");
        }
    }    
}

void loop() { 
   vTaskDelay(10000 / portTICK_PERIOD_MS);
}
