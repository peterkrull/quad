#include <Arduino.h>
#include <FreeRTOS.h>
#include "DShotRMT.h"
#include "sigProc.h"
#include <math.h>

//
// PINOUT DEFINITIONS
//

#define LED_BUILTIN 2

#define M1_pin 26

struct quadcopter_motors{
  int32_t M1 = 0;
} motors;

struct motor {
  uint16_t speed = 0;
  DShotRMT dshot;
  bool reversed = false;
} M1;

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

void vSerialController(void *pvParameters) {
  const TickType_t xFrequency = 100;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, int(1000 / xFrequency));

    motors.M1 = 0;
  }
}

void vMotorGovernor(void *pvParameters) {

  M1.speed = 48;
  bool rmt_init = !(
  M1.dshot.install(gpio_num_t(M1_pin), rmt_channel_t(0)));
  if (rmt_init) {

    xTaskCreate(initMotor, "initM1", 5000, &M1, 1, NULL);
    const TickType_t xFrequency = 10;
    while (true){

      //Serial.println("Looping");

      vTaskDelay(xFrequency / portTICK_PERIOD_MS);

      if (Serial.available()>0){
        String rcv = Serial.readStringUntil(*"\n");
        if (!rcv.equals("")){

          uint8_t times = rcv.toInt();

          if (times > 20) times = 20;

          motors.M1 = 600;
          Serial.print("Setting : ");Serial.println(motors.M1);
          M1.speed = motors.M1;

          for (int i = 0 ; i < times ; i++){
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            motors.M1 = 100 + i*100;
            if (motors.M1 < 48) motors.M1 = 48;
            if (motors.M1 > 2047) motors.M1 = 2047;
            Serial.print("Setting : ");Serial.println(motors.M1);
            M1.speed = motors.M1;
            }

          vTaskDelay(1000 / portTICK_PERIOD_MS);
          motors.M1 = 48;
          Serial.print("Setting : ");Serial.println(motors.M1);
          M1.speed = motors.M1;

        }
      }
      

      // if (Serial.available()>0){

      //   String rcv = Serial.readStringUntil(*"\n");
      //   if (!rcv.equals("")){
          
      //     motors.M1 = 100;
      //     Serial.print("Setting : ");Serial.println(motors.M1);
      //     M1.speed = motors.M1;

      //     vTaskDelay(1000 / portTICK_PERIOD_MS);

      //     motors.M1 = rcv.toInt();

      //     Serial.print("Setting : ");Serial.println(motors.M1);
      //     M1.speed = motors.M1;

      //     vTaskDelay(1500 / portTICK_PERIOD_MS);

      //     motors.M1 = 48;
      //     Serial.print("Setting : ");Serial.println(motors.M1);
      //     M1.speed = motors.M1;
      //   }
      // }
    }
  }
}

//
// Setup and loop
//

void setup() {

  // Initialize LEDs
  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite(LED_BUILTIN,HIGH);
  
  // Initialize serial (USB)
  Serial.begin(115200);
  while (!Serial){
    delay(10);
  } Serial.println("Serial initialized");

  digitalWrite(LED_BUILTIN,LOW);
  
  xTaskCreate(vSerialController,"Controller",2000,NULL,1,NULL);
  xTaskCreate(vMotorGovernor,"motorGovernor",2000,NULL,1,NULL);

  Serial.println("SETUP COMPLETE");

}

void loop() {

  vTaskDelay(1000 / portTICK_PERIOD_MS);

}
