#include "AHRS.h"
#include "Quaternion.h"

AHRS ahrs(53);
unsigned long lastPrint;

void setup() {
    Serial.begin(115200);
    Serial.println("Hello, world!");
    if (!ahrs.initialize()) {
        Serial.println("Error");
        while(1);
    }

    while (millis() < 1000) {
        ahrs.update(false);
    }
}

void loop() {
    static double vX, vY, vZ;
    static unsigned long lastTime = 0;

    bool spun = false;
    while (!ahrs.update(true)) spun = true;

    if (spun) Serial.write('S');
    Serial.write('\t');

    unsigned long currentTime = millis();
    if (lastTime == 0) {
        lastTime = currentTime;
        return;
    }

    double dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    
    Serial.print(ahrs.pitch); Serial.print("\t");
    Serial.print(ahrs.roll); Serial.print("\t");
    Serial.print(ahrs.heading); Serial.print("\t\t");

    double mX = ahrs.xAccel / 4096.0 * 9.8;
    double mY = ahrs.yAccel / 4096.0 * 9.8;
    double mZ = ahrs.zAccel / 4096.0 * 9.8;

    Serial.print(mX); Serial.print("\t");
    Serial.print(mY); Serial.print("\t");
    Serial.print(mZ); Serial.print("\t\t");

    // rotate backwards around accelerometer axis
    Quaternion acceleration(0, mX, mY, mZ);
    acceleration = ahrs.attitude * acceleration * ahrs.attitude.conjugate();

    double aX = acceleration.b;
    double aY = acceleration.c;
    double aZ = acceleration.d;

    Serial.print(aX); Serial.print("\t");
    Serial.print(aY); Serial.print("\t");
    Serial.print(aZ); Serial.print("\t\t");

    aZ -= 9.8;

    vX += aX * dt;
    vY += aY * dt;
    vZ += aZ * dt;

    Serial.print(vX); Serial.print("\t");
    Serial.print(vY); Serial.print("\t");
    Serial.print(vZ); Serial.println();
}
