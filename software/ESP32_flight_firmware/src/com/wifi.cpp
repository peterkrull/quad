#include "wifi.h"
#include <WiFi.h>
#include <AsyncUDP.h>
#include "../board/definitions.h"

wifi::wifi(String SSID,String password) {
    char * ssid;
    char * pass;
    
    SSID.toCharArray(ssid,SSID.length());
    password.toCharArray(pass,password.length());

    xSSID = ssid;
    xPassword = pass;
}

void wifi::vKeepConnection(void *pvParameters){
    // while (true) {
    //     if (WiFi.status() != WL_CONNECTED) {
    //         digitalWrite(LED_BUILTIN,LOW);
    //         WiFi.begin(xSSID,xPassword);
    //         vTaskDelay(5000 / portTICK_PERIOD_MS);
    //     } else {
    //         digitalWrite(LED_BUILTIN,HIGH);
    //     }
    //     vTaskDelay(500 / portTICK_PERIOD_MS);
    // }
}
