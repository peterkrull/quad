// This file defines the pinout and configuration of the prototype PCB developed for this project.

#ifndef MAINBOARD
#define MAINBOARD

    #define MCU_ESP32_WROOM_32E
    #define LOGIC_VOLTAGE 3.3
    #define LED_BUILTIN 0  
    #define LED_RED 33

    #define SBUS_RX 16
    #define SBUS_TX 17

    #define M1_pin 25
    #define M2_pin 27
    #define M3_pin 14
    #define M4_pin 26

    #define GPS_RX 18
    #define GPS_TX 19

    // #define VBAT_pin 1

    #define USE_MPU6050 true
    #define USE_UBLOX_NEO_M6 true

    #define GYROCOMP 939.65

#endif

