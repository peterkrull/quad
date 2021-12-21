#include <Arduino.h>
#include <FreeRTOS.h>
#include "DShotRMT.h"
#include "wifi_credentials.h"
#include "sigProc.h"
#include <math.h>
#include <WiFi.h>
#include "MPU6050_6Axis_MotionApps612.h"

wifi_creds credentials = wifi_creds();
WiFiUDP udp;
const int udp_port = 51000;
const char *udp_addr = "255.255.255.255";

//
// PINOUT DEFINITIONS
//

#define LED_BUILTIN 2
#define LED_GREEN 3
#define LED_YELLOW 4
#define LED_RED 5

#define M1_pin 14
#define M2_pin 27
#define M3_pin 26
#define M4_pin 25

#define GPS_RX 19
#define GPS_TX 18

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

struct IMU_data
{
  int16_t x_accel;
  int16_t y_accel;
  int16_t z_accel;
  int16_t x_gyro;
  int16_t y_gyro;
  int16_t z_gyro;
  float pitch;
  float yaw;
  float roll;
  int16_t temp;
} data_imu;

//
// MOTOR / ECS SETUP
//

DShotRMT M1;
DShotRMT M2;
DShotRMT M3;
DShotRMT M4;

SemaphoreHandle_t M1_ready = xSemaphoreCreateBinary();
SemaphoreHandle_t M2_ready = xSemaphoreCreateBinary();
SemaphoreHandle_t M3_ready = xSemaphoreCreateBinary();
SemaphoreHandle_t M4_ready = xSemaphoreCreateBinary();

//
// GPS / POSITIONING SETUP
//

// Data som det er hentet af GPS-modulet
struct allData
{
  String time;
  String date;
  String latitude;
  String lathem;
  String longitude;
  String longhem;
  char valid;
} pulled;

struct gpsData
{
  char valid;
  int64_t time;
  float latitude;
  float longitude;
} gps_data;

//
// CONTROLLERS
//

PID pid_pitch;

// TODO make single method for motor initialization tasks

void initM1(void *pvParameters)
{
  xSemaphoreTake(M1_ready, 100 / portTICK_PERIOD_MS);
  M1.init(true);
  M1.setReversed(false);
  xSemaphoreGive(M1_ready);
  vTaskDelete(NULL);
}

void initM2(void *pvParameters)
{
  xSemaphoreTake(M2_ready, 100 / portTICK_PERIOD_MS);
  M2.init(true);
  M2.setReversed(true);
  xSemaphoreGive(M2_ready);
  vTaskDelete(NULL);
}

void initM3(void *pvParameters)
{
  xSemaphoreTake(M3_ready, 100 / portTICK_PERIOD_MS);
  M3.init(true);
  M3.setReversed(false);
  xSemaphoreGive(M3_ready);
  vTaskDelete(NULL);
}

void initM4(void *pvParameters)
{
  xSemaphoreTake(M4_ready, 100 / portTICK_PERIOD_MS);
  M4.init(true);
  M4.setReversed(true);
  xSemaphoreGive(M4_ready);
  vTaskDelete(NULL);
}

void vKeepConnection(void *pvParameters)
{
  while (true)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(credentials.SSID, credentials.password);
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

bool read_MPU(struct IMU_data *data)
{

  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gyro, fifoBuffer);

    data->temp = mpu.getTemperature();

    data->x_accel = aaReal.x;
    data->y_accel = aaReal.y;
    data->z_accel = aaReal.z;

    data->x_gyro = gyro.x;
    data->y_gyro = gyro.y;
    data->z_gyro = gyro.z;

    data->pitch = ypr[1];
    data->roll = ypr[2];
    data->yaw = ypr[0];

    return true;
  }
  else
  {
    return false;
  }
}

void vMPUread(void *pvParameters)
{
  const TickType_t xFrequency = 200;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, int(1000 / xFrequency));
    read_MPU(&data_imu);
  }
}

void vTelemetry(void *pvParameters)
{

  if (WiFi.status() == WL_CONNECTED)
  {
    udp.begin(udp_port);
  }

  const TickType_t xFrequency = 10;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      vTaskDelayUntil(&xLastWakeTime, int(1000 / xFrequency));
      uint32_t time_ms = millis();
      digitalWrite(LED_BUILTIN, HIGH);
      udp.beginPacket(udp_addr, udp_port);
      udp.write((const uint8_t *)&time_ms, sizeof(time_ms));
      udp.write((const uint8_t *)&data_imu, sizeof(data_imu));
      udp.write((const uint8_t *)&gps_data, sizeof(gps_data));
      udp.endPacket();
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

// Pulls the GPS data from the module into a string struct.
void gpsPull(struct allData *output)
{

  // Flush serial connection and read available output
  Serial1.flush();
  while (Serial1.available() > 0)
  {
    Serial1.read();
  }

  // If GPRMC is not present, wait for 50 ms
  if (!Serial1.find("$GPRMC,"))
  {
    vTaskDelay(50 / portTICK_PERIOD_MS);

    // If GPRMC is present, split the string
  }
  else
  {

    int pos = 0;
    String tempMsg = Serial1.readStringUntil('\n'); // GPS output for $GPRMC linjen placeres i en tempMsg string
    int stringplace = 0;                            // Variabel til at holde den læste position i GPS-output
    String nmea[9];                                 // String array til at holde hvert formateret output
    char buf[3];

    for (int i = 0; i < tempMsg.length(); i++)
    {
      if (tempMsg.substring(i, i + 1) == ",")
      {
        nmea[pos] = tempMsg.substring(stringplace, i);
        stringplace = i + 1;
        pos++;
      }
      if (i == tempMsg.length() - 1)
      {
        nmea[pos] = tempMsg.substring(stringplace, i);
      }
    }
    nmea[1].toCharArray(buf, 3);

    output->time = nmea[0];      // time
    output->date = nmea[8];      // date
    output->latitude = nmea[2];  // latitude
    output->lathem = nmea[3];    // lat hem
    output->longitude = nmea[4]; // longitude
    output->longhem = nmea[5];   // long hem
    output->valid = buf[0];      // validity
  }
}

// Formats the GPS data for sending. From string to string
void formatForSending(struct allData *input, struct gpsData *output)
{
  //              SECTION FOR FORMATTING TIME             //
  //                    YY-MM-DD-HH-MM                    //
  // Raw date format is : DD-MM-YY
  // Raw time format is : HH-MM-SS
  String day, mon, yea;
  String hou, min, sec;

  day = input->date.substring(0, 2);
  mon = input->date.substring(2, 4);
  yea = input->date.substring(4, 6);

  hou = input->time.substring(0, 2);
  min = input->time.substring(2, 4);
  sec = input->time.substring(4, 6);

  String dateString = yea + mon + day + hou + min;

  output->time = dateString.toInt();

  //            SECTION FOR FORMATTING LATITUDE           //
  //   1800000000 - H is 1 for negative, 2 for positive  // 10 karakterer

  String latfirst, latsecond, latString, lathem;
  latfirst = input->latitude.substring(0, 4);
  latsecond = input->latitude.substring(5, 10);

  if (input->lathem == "N")
  {
    lathem = "+";
  }
  else if (input->lathem == "S")
  {
    lathem = "-";
  }
  latString = latfirst + latsecond + lathem;
  output->latitude = latString.toFloat();

  //            SECTION FOR FORMATTING LONGITUDE          //
  //    900000000H - H is 1 for negative, 2 for positive  // 10 karakterer

  String longfirst, longsecond, longString, longhem;
  longfirst = input->longitude.substring(0, 5);
  longsecond = input->longitude.substring(6, 11);

  if (input->longhem == "E")
  {
    longhem = "+";
  }
  else if (input->longhem == "W")
  {
    longhem = "-";
  }
  longString = longfirst + longsecond + longhem;
  output->longitude = longString.toFloat();
  output->valid = input->valid;
}

void vGpsPull(void *pvParameters)
{

  while (true)
  {
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Data fra GPS-modulet hentes og leveres i allData struct til nem aflæsning
    gpsPull(&pulled);

    // Det afhentede data kan nu formateres
    formatForSending(&pulled, &gps_data);
  }
}

// https://andydoz.blogspot.com/2016/08/automatic-configuration-of-ublox-m6-gps.html
void GPSSerialInit()
{
  Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); // start the comms with the GPS Rx
  delay(6000);                                    // allow the u-blox receiver to come up
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

uint8_t DShotInit()
{
  Serial.println("INSTALLING RMT");
  uint8_t fails = 0;
  if (M1.install(gpio_num_t(M1_pin), rmt_channel_t(0)))
  {
    Serial.println("Motor 1 RMT failed to initialize");
    fails++;
  }

  if (M2.install(gpio_num_t(M2_pin), rmt_channel_t(1)))
  {
    Serial.println("Motor 2 RMT failed to initialize");
    fails++;
  }

  if (M3.install(gpio_num_t(M3_pin), rmt_channel_t(2)))
  {
    Serial.println("Motor 3 RMT failed to initialize");
    fails++;
  }

  if (M4.install(gpio_num_t(M4_pin), rmt_channel_t(3)))
  {
    Serial.println("Motor 4 RMT failed to initialize");
    fails++;
  }

  if (fails > 0)
  {
    Serial.println("INITIALIZING MOTORS");
    xTaskCreate(initM1, "initM1", 1000, NULL, 1, NULL);
    xTaskCreate(initM2, "initM2", 1000, NULL, 1, NULL);
    xTaskCreate(initM3, "initM3", 1000, NULL, 1, NULL);
    xTaskCreate(initM4, "initM4", 1000, NULL, 1, NULL);
  }

  return fails;
}

void setup()
{

  pid_pitch = PID(1,1,1);


  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  // Initialize MPU
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // Initialize GPS and dshot ECSs
  GPSSerialInit();
  uint8_t dshotStatus = DShotInit();

  // Catch unsuccesful initializations
  if (devStatus == 0 && dshotStatus == 0)
  {

    mpu.CalibrateAccel(5);
    mpu.CalibrateGyro(5);
    mpu.setDMPEnabled(true);
    mpu.setDLPFMode(MPU6050_DLPF_BW_98);
    packetSize = mpu.dmpGetFIFOPacketSize();

    xTaskCreate(vKeepConnection, "KeepConnection", 4000, NULL, 2, NULL);
    xTaskCreate(vMPUread, "vMPUread", 4000, NULL, 1, NULL);
    xTaskCreate(vGpsPull, "GpsPull", 2000, NULL, 4, NULL);

    Serial.println("SETUP COMPLETE");
  }
  else
  {
    Serial.println("SETUP FAILED");
    while (1)
    {
      delay(10000);
    }
  }
}

void loop()
{

  // Generate test-signal as sine wave
  int speed = (sin(double(micros()) / 2000000) + 1) * 200 + 100;

  Serial.println(speed);
  if (xSemaphoreTake(M1_ready, 100 / portTICK_PERIOD_MS))
  {
    M1.sendThrottle(speed);
    xSemaphoreGive(M1_ready);
  }

  if (xSemaphoreTake(M2_ready, 100 / portTICK_PERIOD_MS))
  {
    M2.sendThrottle(speed + 50);
    xSemaphoreGive(M2_ready);
  }

  if (xSemaphoreTake(M3_ready, 100 / portTICK_PERIOD_MS))
  {
    M3.sendThrottle(speed + 100);
    xSemaphoreGive(M3_ready);
  }

  if (xSemaphoreTake(M4_ready, 100 / portTICK_PERIOD_MS))
  {
    M4.sendThrottle(speed);
    xSemaphoreGive(M4_ready);
  }
}
