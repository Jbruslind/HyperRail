#include <K30.h>

// Open a sensor connection on pins 2 and 3 (RX/TX)
K30 k30_sensor(2, 3);

void setup(){
    Serial.begin(9600);
    k30_sensor.initSensor();
}

void loop(){
    Serial.println(k30_sensor.getCO2Reading());
    delay(20);
}