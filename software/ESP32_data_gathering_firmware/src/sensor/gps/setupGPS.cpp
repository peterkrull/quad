#include"setupGPS.h"
#include<Arduino.h>

void GPSSerialInit(HardwareSerial serial,uint8_t rx,uint8_t tx) {
  // https://andydoz.blogspot.com/2016/08/automatic-configuration-of-ublox-m6-gps.html
  serial.begin(9600, SERIAL_8N1, rx, tx); // start the comms with the GPS Rx
  delay(1000);                                    // allow the u-blox receiver to come up
  // send serial to update u-blox rate to 200mS
  serial.write(0xB5);
  serial.write(0x62);
  serial.write(0x06);
  serial.write(0x08);
  serial.write(0x06);
  serial.write(0x00);
  serial.write(0xC8);
  serial.write(0x00);
  serial.write(0x01);
  serial.write(0x00);
  serial.write(0x01);
  serial.write(0x00);
  serial.write(0xDE);
  serial.write(0x6A);
  serial.write(0xB5);
  serial.write(0x62);
  serial.write(0x06);
  serial.write(0x08);
  serial.write(0x00);
  serial.write(0x00);
  serial.write(0x0E);
  serial.write(0x30);
  delay(100);
  serial.flush();
  // set 57,600 baud on u-blox
  serial.write(0xB5);
  serial.write(0x62);
  serial.write(0x06);
  serial.write(0x00);
  serial.write(0x14);
  serial.write(0x00);
  serial.write(0x01);
  serial.write(0x00);
  serial.write(0x00);
  serial.write(0x00);
  serial.write(0xD0);
  serial.write(0x08);
  serial.write(0x00);
  serial.write(0x00);
  serial.write(0x00);
  serial.write(0xE1);
  serial.write(0x00);
  serial.write(0x00);
  serial.write(0x07);
  serial.write(0x00);
  serial.write(0x02);
  serial.write(0x00);
  serial.write(0x00);
  serial.write(0x00);
  serial.write(0x00);
  serial.write(0x00);
  serial.write(0xDD);
  serial.write(0xC3);
  serial.write(0xB5);
  serial.write(0x62);
  serial.write(0x06);
  serial.write(0x00);
  serial.write(0x01);
  serial.write(0x00);
  serial.write(0x01);
  serial.write(0x08);
  serial.write(0x22);
  delay(100);
  serial.end(); // stop serial coms at 9,600 baud
  delay(100);
  serial.begin(57600, SERIAL_8N1, rx, tx); // start serial coms at 57,600 baud.
}
