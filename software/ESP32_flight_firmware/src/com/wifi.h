#ifndef wifi_h
#define wifi_h

#include "wifi_credentials.h"
#include <Arduino.h>

#ifdef ESP32
class wifi {
    public:
        wifi(String SSID,String password);
        void vKeepConnection(void *pvParameters);
    private:
        const char * xSSID;
        const char * xPassword;
};
#endif

#endif