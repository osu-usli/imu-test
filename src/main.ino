#include "AHRS.h"

AHRS ahrs;
unsigned long lastPrint;

void setup() {
    Serial.begin(115200);
    Serial.println("Hello, world!");
    if (ahrs.initialize() != 0) {
        Serial.println("Error");
        while(1);
    }
}

void loop() {
    unsigned long currentTime = millis();
    if (ahrs.update()) {
        Serial.print(ahrs.pitch);
        Serial.print("\t");
        Serial.print(ahrs.roll);
        //Serial.print("\t");
        //Serial.print(ahrs.pitchRate);
        //Serial.print("\t");
        //Serial.print(ahrs.rollRate);
        Serial.println();
        lastPrint = currentTime;
    } else {
            Serial.print(" ");
        }
}
