/**
 * Inteface with K30 CO2 sensor, for conducting basic data reads
 * Author: Will Richards
 */

#include "Arduino.h"
#include "SoftwareSerial.h"

#include "K30.h"

/**
 * Construct a new CO2 object
 */
K30::K30(int rx, int tx) : K_30_Serial(rx, tx) {}

/**
 * Start reading data from the sensor
 */
void K30::initSensor()
{
    K_30_Serial.begin(9600);
}

/**
 * Attempt to send the request packet to the sensor
 */ 
unsigned long K30::getCO2Reading() {
    byte response[] = {0,0,0,0,0,0,0};
    byte readCO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};

    while (!K_30_Serial.available()) {
        K_30_Serial.write(readCO2, 7);
        delay(50);
            
    }

    int timeout = 0;                    
    while (K_30_Serial.available() < 7) //Wait to get a 7 byte response
    {
        timeout++;
        if (timeout > 10) //if it takes too long there was probably an error
        {
            while (K_30_Serial.available()) //flush whatever we have
                K_30_Serial.read();
            break; //exit and try again
        }
        delay(50);
    }
    for (int i = 0; i < 7; i++)
    {
        response[i] = K_30_Serial.read();
    }

    return K30::getValue(response);
}

/**
 * Convert the sensor value into an actual value
 */ 
unsigned long K30::getValue(byte packet[])
{
    int high = packet[3]; //high byte for value is 4th byte in packet in the packet
    int low = packet[4]; //low byte for value is 5th byte in the packet
    unsigned long val = high*256 + low; //Combine high byte and low byte with this formula to get value
    return val * 3;
} 