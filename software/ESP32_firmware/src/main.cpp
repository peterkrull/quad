#include <Arduino.h>
#include <FreeRTOS.h>
#include "DShotRMT.h"
#include <math.h>

#define M1_pin 14
#define M2_pin 27
#define M3_pin 26
#define M4_pin 25

DShotRMT M1;
DShotRMT M2;
DShotRMT M3;
DShotRMT M4;

#define LED_BUILTIN 2

SemaphoreHandle_t M1_ready = xSemaphoreCreateBinary();
SemaphoreHandle_t M2_ready = xSemaphoreCreateBinary();
SemaphoreHandle_t M3_ready = xSemaphoreCreateBinary();
SemaphoreHandle_t M4_ready = xSemaphoreCreateBinary();

// TODO make single method for motor initialization tasks

void initM1( void * pvParameters){
  xSemaphoreTake(M1_ready, 100 / portTICK_PERIOD_MS);
  M1.init(true);
  M1.setReversed(false);
  xSemaphoreGive(M1_ready);
  vTaskDelete( NULL );
}

void initM2( void * pvParameters){
  xSemaphoreTake(M2_ready, 100 / portTICK_PERIOD_MS);
  M2.init(true);
  M2.setReversed(true);
  xSemaphoreGive(M2_ready);
  vTaskDelete( NULL );
}

void initM3( void * pvParameters){
  xSemaphoreTake(M3_ready, 100 / portTICK_PERIOD_MS);
  M3.init(true);
  M3.setReversed(false);
  xSemaphoreGive(M3_ready);
  vTaskDelete( NULL );
}

void initM4( void * pvParameters){
  xSemaphoreTake(M4_ready, 100 / portTICK_PERIOD_MS);
  M4.init(true);
  M4.setReversed(true);
  xSemaphoreGive(M4_ready);
  vTaskDelete( NULL );
}

// https://andydoz.blogspot.com/2016/08/automatic-configuration-of-ublox-m6-gps.html
void GPSInit(){
  Serial.begin(9600);// start the comms with the GPS Rx
  delay (6000); // allow the u-blox receiver to come up
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
  delay (100);
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
  delay (100);
  Serial1.end();// stop Serial1 coms at 9,600 baud
  delay (100);
  Serial1.begin (57600); // start Serial1 coms at 57,600 baud.  
}

void DShotInit(){
  Serial.println("INSTALLING RMT");
  uint8_t fails = 0;
  if ( M1.install(gpio_num_t(M1_pin),rmt_channel_t(0))) {
    Serial.println("Motor 1 RMT failed to initialize");
    fails ++ ;}

  if ( M2.install(gpio_num_t(M2_pin),rmt_channel_t(1))) {
    Serial.println("Motor 2 RMT failed to initialize");
    fails ++ ;}

  if ( M3.install(gpio_num_t(M3_pin),rmt_channel_t(2))) {
    Serial.println("Motor 3 RMT failed to initialize");
    fails ++ ;}

  if ( M4.install(gpio_num_t(M4_pin),rmt_channel_t(3))) {
    Serial.println("Motor 4 RMT failed to initialize");
    fails ++ ;}

  if (fails > 0){
    Serial.println("INITIALIZING MOTORS");
    xTaskCreate(initM1,"initM1",1000,NULL,1,NULL);
    xTaskCreate(initM2,"initM2",1000,NULL,1,NULL);
    xTaskCreate(initM3,"initM3",1000,NULL,1,NULL);
    xTaskCreate(initM4,"initM4",1000,NULL,1,NULL);
  }
}

void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  Serial.begin(115200);

  GPSInit();
  DShotInit();

  Serial.println("SETUP COMPLETE");
}


void loop() {

  // Generate test-signal as sine wave
  int speed = (sin(double(micros())/2000000)+1)*200+100;

  Serial.println(speed);
  if (xSemaphoreTake(M1_ready, 100 / portTICK_PERIOD_MS)) {
    M1.sendThrottle(speed);
    xSemaphoreGive(M1_ready);}

  if (xSemaphoreTake(M2_ready, 100 / portTICK_PERIOD_MS)) {
    M2.sendThrottle(speed+50);
    xSemaphoreGive(M2_ready);}

  if (xSemaphoreTake(M3_ready, 100 / portTICK_PERIOD_MS)) {
    M3.sendThrottle(speed+100);
    xSemaphoreGive(M3_ready);}

  if (xSemaphoreTake(M4_ready, 100 / portTICK_PERIOD_MS)) {
    M4.sendThrottle(speed);
    xSemaphoreGive(M4_ready);}
}
