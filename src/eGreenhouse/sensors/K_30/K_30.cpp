#include "Arduino.h"
#include "K_30.h"
#include "SoftwareSerial.h"

K_30::K_30(int rxPin, int txPin){
    _rxPin = rxPin;
    _txPin = txPin;
    _K30_Serial(_rxPin, _txPin);
}

float getCO2(char format){
    
}


