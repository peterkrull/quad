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

wifi_creds credentials = wifi_creds();
AsyncUDP udp;
const int mm_udp_port = 51000;
const char *udp_addr = "255.255.255.255";

//
// PINOUT DEFINITIONS
//

#define LED_BUILTIN 2
#define LED_GREEN 35
#define LED_YELLOW 32
#define LED_RED 33

#define M1_pin 26
#define M2_pin 27
#define M3_pin 14
#define M4_pin 25

#define GPS_RX 18
#define GPS_TX 19

//
// MPU / IMU SETUP
//

MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
VectorInt16 gyro;    // [x, y, z]            gyroscope vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

struct quadcopter_motors{
  int32_t M1 = 0;
  int32_t M2 = 0;
  int32_t M3 = 0;
  int32_t M4 = 0;
} motors,governor;

struct pryt{
  float thrust = 0;
  float pitch = 0;
  float roll = 0;
  float yaw = 0;
} pryt_sp;

struct motor {
  uint16_t speed = 0;
  DShotRMT dshot;
  bool reversed = false;
} M1, M2, M3, M4;

// GPS / POSITIONING SETUP
TinyGPSPlus gps;

// CONTROLLERS
PID pid_pitch, pid_roll, pid_yaw, pid_thrust;

// TODO make single method (if possible) for motor initialization tasks

void initMotor(void *pvParameters) {
  motor M = *((motor*)pvParameters);
  M.dshot.init(true);
  M.dshot.setReversed(M.reversed);
  const TickType_t xFrequency = 200;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, int(1000 / xFrequency));
    uint16_t speed = ((motor*)pvParameters)->speed;
    M.dshot.sendThrottle(speed);
  }
}

void vKeepConnection(void *pvParameters) {
  while (true)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      digitalWrite(LED_BUILTIN,LOW);
      WiFi.begin(credentials.SSID, credentials.password);
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    } else {
      digitalWrite(LED_BUILTIN,HIGH);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void vMPUread(void *pvParameters) {
  const TickType_t xFrequency = 200;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, int(1000 / xFrequency));
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetGyro(&gyro, fifoBuffer);
    }
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

void parseMMUDP(AsyncUDPPacket &packet){
  pryt_sp.pitch = packet.readStringUntil(*"\n").toFloat();
}

void vCommander(void *pvParameters) {
  const TickType_t xFrequency = 10;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, int(1000 / xFrequency));
    if (WiFi.status() == WL_CONNECTED)
    {
      if (udp.listen(mm_udp_port)){
        udp.onPacket(parseMMUDP);
      }
    }
  }
}

void vTinyGPSpull(void *pvParameters){
  while(true){
    vTaskDelay(10 / portTICK_PERIOD_MS);
    uint32_t start_time = micros();
    while (Serial1.available() > 0){
      if (gps.encode(Serial1.read())){
        digitalWrite(LED_RED,gps.location.isValid());
      }
    }
  }
}

void GPSSerialInit() {
  // https://andydoz.blogspot.com/2016/08/automatic-configuration-of-ublox-m6-gps.html
  Serial1.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); // start the comms with the GPS Rx
  delay(1000);                                    // allow the u-blox receiver to come up
  // send Serial1 to update u-blox rate to 200mS
  Serial1.write(0xB5);
  Serial1.write(0x62);
  Serial1.write(0x06);
  Serial1.write(0x08);
  Serial1.write(0x06);
  Serial1.write(0x00);
  Serial1.write(0xC8);
  Serial1.write(0x00);
  Serial1.write(0x01);
  Serial1.write(0x00);
  Serial1.write(0x01);
  Serial1.write(0x00);
  Serial1.write(0xDE);
  Serial1.write(0x6A);
  Serial1.write(0xB5);
  Serial1.write(0x62);
  Serial1.write(0x06);
  Serial1.write(0x08);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0x0E);
  Serial1.write(0x30);
  delay(100);
  Serial1.flush();
  // set 57,600 baud on u-blox
  Serial1.write(0xB5);
  Serial1.write(0x62);
  Serial1.write(0x06);
  Serial1.write(0x00);
  Serial1.write(0x14);
  Serial1.write(0x00);
  Serial1.write(0x01);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0xD0);
  Serial1.write(0x08);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0xE1);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0x07);
  Serial1.write(0x00);
  Serial1.write(0x02);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0xDD);
  Serial1.write(0xC3);
  Serial1.write(0xB5);
  Serial1.write(0x62);
  Serial1.write(0x06);
  Serial1.write(0x00);
  Serial1.write(0x01);
  Serial1.write(0x00);
  Serial1.write(0x01);
  Serial1.write(0x08);
  Serial1.write(0x22);
  delay(100);
  Serial1.end(); // stop Serial1 coms at 9,600 baud
  delay(100);
  Serial1.begin(57600, SERIAL_8N1, GPS_RX, GPS_TX); // start Serial1 coms at 57,600 baud.
}

low_pass thrust_lp = low_pass(0.4);
low_pass pitch_lp = low_pass(0.4);
low_pass roll_lp = low_pass(0.4);
low_pass yaw_lp = low_pass(0.4);

void vController(void *pvParameters) {
  const TickType_t xFrequency = 100;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, int(1000 / xFrequency));


    double con_pitch = pid_pitch.update(ypr[2]-pitch_lp.update(pryt_sp.pitch));
    // double con_roll = pid_roll.update(ypr[1]-pryt_sp.roll);
    // double con_yaw = pid_yaw.update(ypr[0]-pryt_sp.yaw);

    // motors.M1 = pryt_sp.thrust - con_pitch + con_roll + con_yaw;
    // motors.M2 = pryt_sp.thrust + con_pitch - con_roll + con_yaw;
    // motors.M3 = pryt_sp.thrust - con_pitch - con_roll - con_yaw;
    // motors.M4 = pryt_sp.thrust + con_pitch + con_roll - con_yaw;

    motors.M1 = 300 - con_pitch;
    motors.M3 = 300 - con_pitch;

  }
}

void vMotorGovernor(void *pvParameters) {

  M1.speed = 100;
  M2.speed = 0;
  M3.speed = 100;
  M4.speed = 0;

  M3.reversed = true;
  M4.reversed = true;

  bool rmt_init = !(
  M1.dshot.install(gpio_num_t(M1_pin), rmt_channel_t(0)) &
  M2.dshot.install(gpio_num_t(M2_pin), rmt_channel_t(1)) &
  M3.dshot.install(gpio_num_t(M3_pin), rmt_channel_t(2)) &
  M4.dshot.install(gpio_num_t(M4_pin), rmt_channel_t(3)) );

  if (rmt_init) {
    xTaskCreate(initMotor, "initM1", 5000, &M1, 1, NULL);
    xTaskCreate(initMotor, "initM2", 5000, &M2, 1, NULL);
    xTaskCreate(initMotor, "initM3", 5000, &M3, 1, NULL);
    xTaskCreate(initMotor, "initM4", 5000, &M4, 1, NULL);

    const TickType_t xFrequency = 200;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
      vTaskDelayUntil(&xLastWakeTime, int(1000 / xFrequency));

      if (true){ // If all is good

        if (motors.M1 < 48) motors.M1 = 48;
        if (motors.M2 < 48) motors.M2 = 48;
        if (motors.M3 < 48) motors.M3 = 48;
        if (motors.M4 < 48) motors.M4 = 48;

        if (motors.M1 > 2047) motors.M1 = 2047;
        if (motors.M2 > 2047) motors.M2 = 2047;
        if (motors.M3 > 2047) motors.M3 = 2047;
        if (motors.M4 > 2047) motors.M4 = 2047;

        M1.speed = motors.M1;
        M2.speed = motors.M2;
        M3.speed = motors.M3;
        M4.speed = motors.M4;

        //Serial.print(M1.speed);Serial.print("\t: "); Serial.print(M2.speed);Serial.print("\t: "); Serial.print(M3.speed);Serial.print("\t: "); Serial.println(M4.speed);
      } else if (true){
        // Secondary failure mode,
      } else { // catastropic failsafe, cut all motors
        M1.speed = 0;
        M2.speed = 0;
        M3.speed = 0;
        M4.speed = 0;
      }
    }
  }
}

//
// Setup and loop
//

void setup() {

  // Initialize LEDs
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN,HIGH);
  pinMode(LED_GREEN,OUTPUT); digitalWrite(LED_GREEN,HIGH);
  pinMode(LED_YELLOW,OUTPUT); digitalWrite(LED_YELLOW,HIGH);
  pinMode(LED_RED,OUTPUT); digitalWrite(LED_RED,HIGH);
  
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
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0){
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    mpu.setDLPFMode(MPU6050_DLPF_BW_98);
    packetSize = mpu.dmpGetFIFOPacketSize();
  } Serial.println("devStatus received");

  // Initialize GPS
  GPSSerialInit();
  Serial.println("GPS initializd");

  digitalWrite(LED_BUILTIN,LOW); digitalWrite(LED_GREEN,LOW); digitalWrite(LED_YELLOW,LOW); digitalWrite(LED_RED,LOW);

  // Catch unsuccesful initializations
  if (devStatus == 0) {

    // initialize pryt controllers
    pid_thrust = PID(1,0,0);
    pid_pitch = PID(200,100,55);
    pid_roll = PID(200,300,50);
    pid_yaw = PID(1,0,0);
    
    xTaskCreate(vKeepConnection, "KeepConnection", 3000, NULL, 4, NULL);
    xTaskCreate(vCommander, "Commander", 2000, NULL, 3, NULL);
    xTaskCreate(vMPUread, "vMPUread", 4000, NULL, 2, NULL);
    xTaskCreate(vTinyGPSpull, "tinyGPSpull", 2000, NULL, 3, NULL);
    xTaskCreate(vController,"Controller",2000,NULL,1,NULL);
    xTaskCreate(vMotorGovernor,"motorGovernor",2000,NULL,1,NULL);

    Serial.println("SETUP COMPLETE");
  }
  else {
    Serial.println("SETUP FAILED");
  }
}

void loop() {

  vTaskDelay(1000 / portTICK_PERIOD_MS);

}
