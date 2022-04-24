#ifndef boards_h
#define boards_h

#ifdef BOARD_QUAD_V01

    #define LED_BUILTIN 0

    #define M1_pin 26
    #define M2_pin 27
    #define M3_pin 14
    #define M4_pin 25

    #define GPS_RX 18
    #define GPS_TX 19

    #define IO_0 15
    #define IO_1 13
    #define IO_2 16
    #define IO_3 17
    #define IO_4 5
    #define IO_5 16
    #define IO_6 25
    #define IO_7 26
    #define IO_8 26
    #define IO_9 14

    #define USES_MPU6050
    #define USES_UBLOX_NEO

    #define GYROCOMP 939.65

#endif // BOARD_QUAD_V01

#endif