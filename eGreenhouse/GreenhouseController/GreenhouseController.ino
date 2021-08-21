#include <K30.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_TSL2591.h>

// CO2 Sensor
K30 k30 = K30(2, 3);

// Temperature and Humidity sensor (Docs: https://learn.adafruit.com/adafruit-sht31-d-temperature-and-humidity-sensor-breakout/wiring-and-test)
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Light Sensor (Docs: https://learn.adafruit.com/adafruit-tsl2591/wiring-and-test)
Adafruit_TSL2591 ts1 = Adafruit_TSL2591();

void setup(){
    Serial.begin(9600);

    // Init sensors for reading
    k30.initSensor();

    // Set the I2C address for the humidity sensor to 0x44 and start reading
    if(!sht31.begin(0x44)){
        Serial.println("ERROR! Humidity Sensor (SHT31) Did not respond to communication request.");
    }

    // Init the light sensor with medium settings (adjust accordingly), NOTE: This runs on I2C address 0x29/0x28
    ts1.setGain(TSL2591_GAIN_MED);
    ts1.setTiming(TSL2591_INTEGRATIONTIME_100MS);
}

void loop(){
    Serial.println("CO2: " + String(k30.getCO2Reading()));
    Serial.println("Temperature: " + String(sht31.readTemperature()));
    Serial.println("Humdity: " + String(sht31.readHumidity()));
    Serial.println("Luminosity (Visible): " + String(ts1.getLuminosity(TSL2591_VISIBLE)));
    delay(20);
}