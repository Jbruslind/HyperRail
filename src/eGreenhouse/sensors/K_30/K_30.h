/*
    K_30.h - Library to read sensor data from a K30 CO2 Sensor
    Created by Will Richards, July 16, 2021
*/
#ifndef K_30_h
#define K_30_h

#include "Arduino.h"
#include "SoftwareSerial.h"

class K_30{
    public:
        K_30(int rxPin, int txPin);
        float getCO2(char format);
        float getTemp();
        float getRH();
    private:
        int _rxPin;
        int _txPin;
        SoftwareSerial _K30_Serial;
};

#endif