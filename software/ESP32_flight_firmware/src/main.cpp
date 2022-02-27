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
// SYSTEM CONSTANTS
//

#define gyroradss 939.65

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
  int16_t M1 = 0;
  int16_t M2 = 0;
  int16_t M3 = 0;
  int16_t M4 = 0;
} motors;

struct pryt{
  float thrust = 0;
  float pitch = 0;
  float roll = 0;
  float yaw = 0;
} pryt_sp;

struct motor {
  uint16_t speed = 48;
  DShotRMT dshot;
  bool reversed;
  bool rmt_init = false;
  uint8_t rmt_channel;
  uint8_t gpio_pin;
  bool standby = true;
  bool ready = false;
} M1, M2, M3, M4;

// GPS / POSITIONING SETUP
TinyGPSPlus gps;

// CONTROLLERS
PID pid_pitch, pid_roll, pid_yaw, pid_thrust, pid_vpitch, pid_vroll, pid_vyaw, pid_vthrust;

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

bool motors_enable = false;
uint16_t hoverthrust = 730;
int16_t offset = 0;

#define udpTimeoutMS 500
int32_t udpTimeoutCntr = 0;

void parseUDP(AsyncUDPPacket &packet){
  uint32_t xmillis = millis();
  
  // Heartbeat time out detections
  if (udpTimeoutCntr == 0){
    udpTimeoutCntr = xmillis;
  } else {
    uint16_t UDPdeltaTime = xmillis - udpTimeoutCntr;
    udpTimeoutCntr = xmillis;
  } if (udpTimeoutCntr > udpTimeoutMS) {motors_enable = 0;}
  
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

esp_err_t sendMotorThrottle(motor *M, uint16_t speed) {
  if (speed < 48) speed = 48; if (speed > 2047) speed = 2047;
  if (motors_enable){
    ((motor*)M)->dshot.sendThrottle(speed);
  } else {
    ((motor*)M)->dshot.sendThrottle(48);
  }
}


low_pass thrust_lp = low_pass(0.1);
low_pass pitch_lp = low_pass(0.1);
low_pass roll_lp = low_pass(0.1);
low_pass yaw_lp = low_pass(0.1);

void getAllMPU(){
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.dmpGetGyro(&gyro, fifoBuffer);
}

void motorMixing(quadcopter_motors *motor, uint16_t thrust, uint16_t pitch, uint16_t roll, uint16_t  yaw){
  motor -> M1 = thrust + pitch - roll - yaw;
  motor -> M2 = thrust - pitch + roll - yaw;
  motor -> M3 = thrust + pitch + roll + yaw;
  motor -> M4 = thrust - pitch - roll + yaw;
}


void vHyperLoop(void *pvParameters){

  while (true) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      
      getAllMPU();

      double con_pitch = pid_pitch.update(pryt_sp.pitch-ypr[2]);
      double con_vpitch = pid_vpitch.update(pryt_sp.pitch-(gyro.x/gyroradss)-(float(offset)/1000));

      // double con_roll = pid_roll.update(pryt_sp.roll-ypr[1]);
      // double con_vroll = pid_vroll.update(pryt_sp.roll-(gyro.y/gyroradss)-(float(offset)/1000));
      // double con_yaw = pid_yaw.update(ypr[0]-pryt_sp.yaw);

      motorMixing(&motors, hoverthrust, con_vpitch, 0, 0);

      if (sanityCheck()){
        sendMotorThrottle(&M1,motors.M1);
        sendMotorThrottle(&M2,motors.M2);
        sendMotorThrottle(&M3,motors.M3);
        sendMotorThrottle(&M4,motors.M4);
      }

      vTaskDelay(2 / portTICK_PERIOD_MS);
    }
  }
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
  if (abs(ypr[1]) > PI/2 || abs(ypr[2]) > PI/2) {return;}

  // Check for prolonged total motor power
  if (totalmotorsanity_lp.update(motors.M1+motors.M2+motors.M3+motors.M4)/4 > 1000) {return;}

  // Check for proloneged individual motor power
  if (M1sanity_lp.update(motors.M1) > 1500) {return;}
  if (M2sanity_lp.update(motors.M2) > 1500) {return;}
  if (M3sanity_lp.update(motors.M3) > 1500) {return;}
  if (M3sanity_lp.update(motors.M4) > 1500) {return;}

  // Check for abnormal acceleration values
  if (AAsanity_lp.update(aaReal.getMagnitude()) > 5000) {return;}

  motors_enable = 1;

  return 1;
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

  // Set motor configurations
  M1.reversed = true;   M1.gpio_pin = M1_pin;   M1.rmt_channel = 0;
  M2.reversed = true;   M2.gpio_pin = M2_pin;   M2.rmt_channel = 1;
  M3.reversed = false;  M3.gpio_pin = M3_pin;   M3.rmt_channel = 2;
  M4.reversed = false;  M4.gpio_pin = M4_pin;   M4.rmt_channel = 3;
  
  // Create motor initialization tasks
  xTaskCreate(motorInit, "motorGovM1", 5000, &M1, 1, NULL);
  xTaskCreate(motorInit, "motorGovM2", 5000, &M2, 1, NULL);
  xTaskCreate(motorInit, "motorGovM3", 5000, &M3, 1, NULL);
  xTaskCreate(motorInit, "motorGovM4", 5000, &M4, 1, NULL);

  mpu.initialize();
  Serial.println("MPU Initialized");
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0){
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    mpu.setDLPFMode(MPU6050_DLPF_BW_256);
    packetSize = mpu.dmpGetFIFOPacketSize();
  } Serial.println("devStatus received");

  // Initialize GPS
  GPSSerialInit();
  Serial.println("GPS initializd");

  // Set all LEDs low
  digitalWrite(LED_BUILTIN,LOW); digitalWrite(LED_GREEN,LOW);
  digitalWrite(LED_YELLOW,LOW); digitalWrite(LED_RED,LOW);

  // Catch unsuccesful initializations
  if (devStatus == 0) {

    // initialize pryt controllers      // Velocity pryt controllers
    pid_thrust = PID(1,0,0);            pid_vthrust = PID(1,0,0);
    pid_pitch = PID(10,0,0.05,0,true);  pid_vpitch = PID(30,0,0.0,0,true);
    pid_roll = PID(10,0,0.05,0,true);   pid_vroll = PID(30,0,0.0,0,true);
    pid_yaw = PID(1,0,0);               pid_vyaw = PID(1,0,0);

    // Create initial system tasks
    xTaskCreate(vKeepConnection, "KeepConnection", 3000, NULL, 4, NULL);
    xTaskCreate(vUDPlistener, "Commander", 2000, NULL, 3, NULL);
    xTaskCreate(vTinyGPSpull, "tinyGPSpull", 2000, NULL, 3, NULL);
    xTaskCreate(vMotorInit,"MotorInit",1000,NULL,2,NULL);

    Serial.println("SETUP COMPLETE");
  }
  else {
    Serial.println("SETUP FAILED");
  }
}

void loop() { vTaskDelay(100000 / portTICK_PERIOD_MS); }
