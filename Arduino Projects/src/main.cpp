#include <Wire.h>
#include <Arduino.h>
#include "Plotter.h"
#include <Kalman.h>
#include <I2C.h>

Kalman kalmanX;
I2C i2c;
Plotter p;

#define RESTRICT_PITCH

double compAngleX, gyroX, gyroXangle, accX, kalAngleX;

double accY, gyroY;

double accZ, gyroZ;

int16_t tempRaw;

double totalAccel = 0.0;

uint32_t timer;

uint8_t i2cData[14]; // Buffer for I2C data

void setup() {
  // TODO : Do callibration if 
  p.Begin();
  p.AddTimeGraph( "Some title of a graph", 5000, "gyroXangle", gyroXangle, "compAngleX", compAngleX, "kalmanX", kalAngleX);
  Wire.begin();
  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2c.i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2c.i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2c.i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    while (1);
  }
  delay(100);

  while (i2c.i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  kalmanX.setAngle(roll); // Set starting angle
  gyroXangle = roll;
  compAngleX = roll;

  timer = micros();
}

void loop() {
  while (i2c.i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  double roll = atan2(accY, accZ) * RAD_TO_DEG;
  double gyroXrate = gyroX / 131.0;

  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);

  gyroXangle += gyroXrate * dt;
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;

  p.Plot(); 
  delay(2);
}
