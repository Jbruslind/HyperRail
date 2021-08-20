/**
 * K_30.h - Library for interfacing with a K30 CO2 Sensor
 * Created by Will Richards, August 20, 2021
 */

#ifdef K30_h
#define K30_h

    #include "Arduino.h"
    #include "SoftwareSerial.h"

    class K30{
        public:
            K30(int rx, int tx);
            void initSensor();
            unsigned long getCO2Reading();
        private:
            SoftwareSerial K_30_Serial;
            unsigned long getValue(byte packet[]);
    }
#endif