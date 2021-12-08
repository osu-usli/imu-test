#include "AHRS.h"
#include <Arduino.h>

#ifdef MPU_SPI
static const int CS = 53;
static const int BARO_CS = 40;
#include <SPI.h>
#else
#include <Wire.h>
#endif

const float GYRO_PERCENTAGE = 0.999;
//const double ACCEL_SCALE = 1e-3;
const unsigned CALIBRATION_THRESHOLD = 4096*0.14;
const unsigned CALIBRATION_TIME = 3000;

const int ACCEL_BIAS_X = 55;
const int ACCEL_BIAS_Y = -90;
const int ACCEL_BIAS_Z = 533;

AHRS::AHRS() {
    pitch = roll = heading = pitchAccel = rollAccel = yawAccel =
    pitchBias = rollBias = yawBias =
    xAccel = yAccel = zAccel = xGyro = yGyro = zGyro = compassHeading = 0;
}


bool AHRS::initialize() {
#ifdef MPU_SPI
    pinMode(CS, OUTPUT);
    pinMode(BARO_CS, OUTPUT);
    digitalWrite(CS, HIGH);
    digitalWrite(BARO_CS, HIGH);
    pinMode(MOSI, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(SCK, OUTPUT);

    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
#endif
    delay(100);
    //Boot the MPU
    writeMPURegister(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
    delay(100);
    writeMPURegister(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
    delay(100);
    writeMPURegister(MPUREG_PWR_MGMT_1, 0);
    delay(100);

    Serial.print("MPU device ID: ");
    Serial.println(readMPURegister(MPUREG_WHOAMI));
    
    //Configure the clock source
    //if (readMPURegister(MPUREG_PWR_MGMT_1) != BIT_PWR_MGMT_1_CLK_ZGYRO) {
    if (readMPURegister(MPUREG_PWR_MGMT_1) != 0) {
        return false;
    }
    
#ifdef MPU_SPI
    //Disable i2c mode
    writeMPURegister(MPUREG_USER_CTRL, BIT_USER_CTRL_I2C_IF_DIS);
    delay(1);
#endif
    
    //Set the filter to 98Hz
    writeMPURegister(MPUREG_CONFIG, BITS_DLPF_CFG_256HZ_NOLPF2);
    
    //Set the sample rate
    writeMPURegister(MPUREG_SMPLRT_DIV, MPUREG_SMPLRT_500HZ);
    delay(1);
    
    //Set the gyro resolution to 500 degrees per second.
    writeMPURegister(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_1000DPS);
    delay(1);
    
    //Set the accelerometer resolution to 4 units per G
    //According to the ArduPilot source, different revisions have different accelerometer scales.
    byte product = readMPURegister(MPUREG_PRODUCT_ID);
    if (product == MPU6000ES_REV_C4 || product == MPU6000ES_REV_C5 ||
        product == MPU6000_REV_C4 || product == MPU6000_REV_C4) {
        
        writeMPURegister(MPUREG_ACCEL_CONFIG, 1 << 3);
        
    } else {
        writeMPURegister(MPUREG_ACCEL_CONFIG, 1 << 4);
    }
    delay(1);
    
    return true;
}

#ifdef MPU_SPI
void AHRS::beginSPI() {
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS, LOW);
}

void AHRS::endSPI() {
    digitalWrite(CS, HIGH);
    SPI.endTransaction();
}

void AHRS::writeMPURegister(byte reg, byte val) {
    //Serial.print("Writing 0x");
    //Serial.print(val, HEX);
    //Serial.print(" to 0x");
    //Serial.println(reg, HEX);
    
    beginSPI();
    
    SPI.transfer(reg);
    SPI.transfer(val);
    
    endSPI();
}

byte AHRS::readMPURegister(byte reg) {
    beginSPI();
    SPI.transfer(reg | 0x80);
    byte result = SPI.transfer(0x00);
    endSPI();
    
    //Serial.print("Read 0x");
    //Serial.print(result, HEX);
    //Serial.print(" from 0x");
    //Serial.println(reg, HEX);
    return result;
}

#else

void AHRS::setupI2C() {
    Wire.begin();
}

const byte MPU_ADDR = 0x68;
void AHRS::writeMPURegister(byte reg, byte val) {
    //Serial.print("Writing 0x");
    //Serial.print(val, HEX);
    //Serial.print(" to 0x");
    //Serial.println(reg, HEX);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

byte AHRS::readMPURegister(byte reg) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom((int)MPU_ADDR, 1, true);
    byte val = Wire.read();
    //Serial.print("Read 0x");
    //Serial.print(val, HEX);
    //Serial.print(" from 0x");
    //Serial.println(reg, HEX);

    return val;
}

#endif

unsigned AHRS::readMPU16(byte regHigh) {
    //beginSPI();
    
    unsigned result = (readMPURegister(regHigh) << 8) | readMPURegister(regHigh + 1);
    
    //endSPI();
    
    //Serial.print("Read 16-bit 0x");
    //Serial.print(result, HEX);
    //Serial.print(" from 0x");
    //Serial.println(regHigh, HEX);
    return result;
}

void AHRS::read() {
    lastYAccel[bufferIndex] = readMPU16(MPUREG_ACCEL_XOUT_H);
    lastXAccel[bufferIndex] = readMPU16(MPUREG_ACCEL_YOUT_H);
    lastZAccel[bufferIndex] = readMPU16(MPUREG_ACCEL_ZOUT_H);
    
    lastYGyro[bufferIndex] = readMPU16(MPUREG_GYRO_XOUT_H);
    lastXGyro[bufferIndex] = readMPU16(MPUREG_GYRO_XOUT_H);
    lastZGyro[bufferIndex] = readMPU16(MPUREG_GYRO_XOUT_H);
    
    bufferIndex += 1;
    bufferIndex %= 5;
}

bool AHRS::update() {
    if (!(readMPURegister(MPUREG_INT_STATUS) & BIT_RAW_RDY_INT)) {
        //if no new data is available, return
        return false;
    }
    
    //Read inputs
    xAccel = xAccelLPF.update(readMPU16(MPUREG_ACCEL_XOUT_H) - ACCEL_BIAS_X);
    yAccel = yAccelLPF.update(readMPU16(MPUREG_ACCEL_YOUT_H) - ACCEL_BIAS_Y);
    zAccel = zAccelLPF.update(readMPU16(MPUREG_ACCEL_ZOUT_H) - ACCEL_BIAS_Z);
    
    xGyro = xGyroLPF.update(readMPU16(MPUREG_GYRO_XOUT_H));
    yGyro = yGyroLPF.update(readMPU16(MPUREG_GYRO_YOUT_H));
    zGyro = zGyroLPF.update(readMPU16(MPUREG_GYRO_ZOUT_H));
    
    //Calculate pitch, yaw, and roll from the accelerometer
    //Based on: http://engineering.stackexchange.com/a/3350
    //rollAccel = 180 * atan (xAccel/sqrt(pow(yAccel,2) +pow(zAccel,2)))/M_PI;
    //pitchAccel = 180 * atan (yAccel/sqrt(pow(xAccel,2) + pow(zAccel,2)))/M_PI;
    //yawAccel = 180 * atan(zAccel/sqrt(pow(xAccel, 2) + pow(ZAccel, 2)))/M_PI;
    pitchAccel = atan2(
                       (double)yAccel,
                       (double)zAccel
                       ) * 180/M_PI;
    rollAccel = atan2(
                      -(double)xAccel,
                      sqrt(((double)yAccel)*yAccel + ((double)zAccel)*zAccel)
                      ) * 180/M_PI;
    
    unsigned long currentTime = millis();
    unsigned long elapsed = lastUpdate == 0 ? 0 : currentTime - lastUpdate;
    lastUpdate = currentTime;

    timeUntilCalibration -= elapsed;
    long totalAcceleration = sqrt(pow(xAccel, 2) + pow(yAccel, 2) + pow(zAccel, 2));
    //Serial.print("Total acceleration: ");
    //Serial.println(totalAcceleration);
    if (abs(4096 - totalAcceleration) > CALIBRATION_THRESHOLD) {
        //Serial.print("Not calibrating as difference is ");
        //Serial.println(abs(4096 - totalAcceleration));
        
        timeUntilCalibration = CALIBRATION_TIME;
    }
    else if (timeUntilCalibration <= 0) {
        //calibrate the gyro
        //Serial.println();
        //Serial.print("Calibrating AHRS.  Difference is ");
        //Serial.println(abs(4096 - totalAcceleration));
        
        calibrationTime = 10;
        
        pitchBias = rollBias = yawBias = 0;
        
        pitch = pitchAccel;
        roll = rollAccel;
        heading = compassHeading;
        
        timeUntilCalibration = CALIBRATION_TIME;
    }
    
    //Calculate pitch, yaw, and roll from the gyro
    //The gyro resolution is 250 degrees per second
    //that's 65.5*2 units per degree per second
    //that's 6,5500*2 units per degree per millisecond
    pitchRate = (xGyro * (1000.0 / 0x7fff));
    rollRate  = (yGyro * (1000.0 / 0x7fff));
    yawRate   = (zGyro * (1000.0 / 0x7fff));
    if (calibrationTime != 0) {
        pitchBias += pitchRate;
        rollBias += rollRate;
        yawBias += yawRate;
        
        if (--calibrationTime == 0) {
            pitchBias /= 10;
            rollBias /= 10;
            yawBias /= 10;
        }
    } else {
        pitchRate -= pitchBias;
        rollRate -= rollBias;
        yawRate -= yawBias;

        double gyroPitch = pitchRate / 1000 * elapsed;
        double gyroRoll = rollRate / 1000 * elapsed;
        double gyroYaw = yawRate / 1000 * elapsed;

        pitch += gyroPitch;
        roll += gyroRoll;
        heading += gyroYaw;

        pitch = GYRO_PERCENTAGE*pitch + ((1-GYRO_PERCENTAGE) * pitchAccel);
        roll = GYRO_PERCENTAGE*roll + ((1-GYRO_PERCENTAGE) * rollAccel);
    }
    
    while (heading >= 360) heading -= 360;
    while (heading < 0) heading += 360;
    
    return true;
}
