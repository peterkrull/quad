#include "wifi.h"

// void espwifi::vKeepConnection(void *pvParameters){
//     while (true) {
//         if (WiFi.status() != WL_CONNECTED) {
//             digitalWrite(LED_BUILTIN,LOW);
//             WiFi.begin(xSSID,xPassword);
//             vTaskDelay(5000 / portTICK_PERIOD_MS);
//         } else {
//             digitalWrite(LED_BUILTIN,HIGH);
//         }
//         vTaskDelay(500 / portTICK_PERIOD_MS);
//     }
// }