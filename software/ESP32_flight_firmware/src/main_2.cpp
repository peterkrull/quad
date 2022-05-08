// #include <Arduino.h>
// #include "configuration.h"
// #include "easy/easyFlight.h"
// #include "health/power.h"
// #include "sensor/imu.h"
// #include "com/wifi.h"

// // wifi_creds credentials = wifi_creds();
// // AsyncUDP udp;
// // const int mm_udp_port = 51000;
// // const char *udp_addr = "255.255.255.255";

// Battery battery = Battery(12,16.4);

// void setup(){
    
//     Serial.begin(BAUDRATE);

//     if ( battery.isLow() ){
//         Serial.printf("Voltage too low, aborting!\nVoltage : %f V\n",battery.getPercentage());
//         while(1) {sleep(1);}
//     }
// }

// void loop(){}