#pragma once
#include <Arduino.h>

struct ImuSample {
  float ax, ay, az;   // g
  float gx, gy, gz;   // deg/s
  float tempC;
};

// Configures the MPU-6050: 200 Hz, DLPF ~94 Hz, +-8 g, +-500 dps.
// Returns WHO_AM_I (0x68 for genuine parts, clones differ) or -1 on I2C failure.
int imuBegin(int sda, int scl, uint8_t addr);
bool imuRead(ImuSample& s);
