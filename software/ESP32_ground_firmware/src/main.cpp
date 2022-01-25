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

// Self-destructing task
void initM1(void *pvParameters)
{
  vTaskDelete(NULL);
}

// Constant task
void vKeepConnection(void *pvParameters)
{
  while (true)
  {

  }
}


void setup()
{

  Serial.begin(115200);
  while (!Serial){
    delay(10);
  }
  Serial.println("Serial initialized");

  // Setup WiFi hotspot
  WiFi.mode(WIFI_AP);
  WiFi.softAP(credentials.SSID, credentials.password);
  
  xTaskCreate(vKeepConnection, "KeepConnection", 4000, NULL, 2, NULL);
}

void loop()
{
  vTaskDelay(10000 / portTICK_PERIOD_MS);
}
