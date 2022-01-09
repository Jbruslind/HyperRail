//Sensor Libs
#include <K30.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_TSL2591.h>

#include <ArduinoJson.h>

// NOTE: In the future we may want to sync the "time" key in the returned JSON object to a synced time with the main controller

// CO2 Sensor (RX: 2, TX: 3)
K30 k30 = K30(2, 3);

// Temperature and Humidity sensor (Docs: https://learn.adafruit.com/adafruit-sht31-d-temperature-and-humidity-sensor-breakout/wiring-and-test)
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Light Sensor (Docs: https://learn.adafruit.com/adafruit-tsl2591/wiring-and-test)
Adafruit_TSL2591 ts1 = Adafruit_TSL2591();

// Byte Received when new data is requested
int inByte = 0;

// JSON Configuration
const int capacity = 223;
StaticJsonDocument<capacity> doc;


void setup(){
    Serial.begin(9600);

    // Init sensors for reading
    k30.initSensor();
    doc[0]["sensor"] = "K30";

    // Set the I2C address for the humidity sensor to 0x44 and start reading
    if(!sht31.begin(0x44)){
        Serial.println("ERROR! Humidity Sensor (SHT31) Did not respond to communication request.");
    }
    doc[1]["sensor"] = "SHT31";

    // Init the light sensor with medium settings (adjust accordingly), NOTE: This runs on I2C address 0x29/0x28
    ts1.setGain(TSL2591_GAIN_MED);
    ts1.setTiming(TSL2591_INTEGRATIONTIME_100MS);
    doc[2]["sensor"] = "TSL2591";
}

void loop(){

    // Wait until Serial data is received, Single byte (eg. "A") from the controller board and then respond with sensor data
    if(Serial.available() > 0){

        // Read data from serial buffer
        inByte = Serial.read();

        // Get CO2 Data
        doc[0]["time"] = millis();
        doc[0]["value"] = k30.getCO2Reading();
        delay(10);

        //Get Temp and humidity data
        doc[1]["time"] = millis();
        doc[1]["temp"] = sht31.readTemperature();
        doc[1]["humidity"] = sht31.readHumidity();

        delay(10);

        // Get luminosity data
        doc[2]["value"] = ts1.getLuminosity(TSL2591_VISIBLE);
        doc[2]["time"] = millis();


        // Write the serialized data to the Serial bus
        serializeJson(doc, Serial);
    }
}
