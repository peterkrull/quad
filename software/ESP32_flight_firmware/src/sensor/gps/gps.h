#include <Arduino.h>

#if USE_UBLOX_NEO_M6
#include "ublox_gps.h"
#include "TinyGPSPlus.h"

class GPS {
    public:
        double getLongitude() { return gps.location.lng(); }
        double getLatitude() { return gps.location.lat(); }
        double getAltitude() { return gps.altitude.meters(); }
        double getVelocity() { return gps.speed.mps(); }
        unsigned int getTimestamp() { return gps.time.value(); }

        bool isUpdated() { 
            while (((HardwareSerial*)nserial)->available()){
                gps.encode( ((HardwareSerial*)nserial)->read() );
            }
            if (gps.time.value() > prev_age && gps.location.isValid()) {
                prev_age = gps.time.value();
                return true;
            } else {
                prev_age = gps.time.value();
                return false;
            }
        }

        bool initialize(HardwareSerial *serial,uint8_t rx,uint8_t tx) {
            nserial = serial;
            GPSSerialInit(nserial,rx,tx); return true;
        }

    private:
        HardwareSerial * nserial;
        float longitude, latitude;
        unsigned long timestamp;
        unsigned long prev_age = -1;
        TinyGPSPlus gps;
};

#endif