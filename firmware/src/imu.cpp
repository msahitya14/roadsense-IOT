#include "imu.h"
#include <Wire.h>

static uint8_t s_addr = 0x68;
static constexpr float ACC_LSB_PER_G = 4096.0f;  // +-8 g
static constexpr float GYR_LSB_PER_DPS = 65.5f;  // +-500 dps

static bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(s_addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static int readReg(uint8_t reg) {
  Wire.beginTransmission(s_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint16_t)s_addr, (size_t)1, true) != 1) return -1;
  return Wire.read();
}

// The MPU-6050 is not reset when the ESP32 resets (flash, serial monitor, EN button). If that happens
// mid-read, the MPU can keep holding SDA low and the whole bus looks dead until power is cycled.
// Standard recovery: clock SCL until the slave releases SDA, then generate a STOP condition.
String i2cRecover(int sda, int scl) {
  Wire.end();
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  delayMicroseconds(20);
  int sda0 = digitalRead(sda), scl0 = digitalRead(scl);
  int pulses = 0;
  pinMode(scl, OUTPUT_OPEN_DRAIN);
  digitalWrite(scl, HIGH);
  while (digitalRead(sda) == LOW && pulses < 16) {
    digitalWrite(scl, LOW);  delayMicroseconds(10);
    digitalWrite(scl, HIGH); delayMicroseconds(10);
    pulses++;
  }
  pinMode(sda, OUTPUT_OPEN_DRAIN);  // STOP: SDA low -> high while SCL is high
  digitalWrite(sda, LOW);  delayMicroseconds(10);
  digitalWrite(scl, HIGH); delayMicroseconds(10);
  digitalWrite(sda, HIGH); delayMicroseconds(10);
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  delayMicroseconds(20);
  char b[96];
  snprintf(b, sizeof(b), "lines before SDA=%d SCL=%d, %d clock pulses, after SDA=%d SCL=%d", sda0, scl0, pulses,
           digitalRead(sda), digitalRead(scl));
  return b;
}

String i2cScan() {
  String found;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      char b[8];
      snprintf(b, sizeof(b), "0x%02X ", a);
      found += b;
    }
  }
  return found.length() ? found : String("none");
}

int imuBegin(int sda, int scl, uint8_t addr, uint32_t hz) {
  s_addr = addr;
  Wire.begin(sda, scl);   // a second call is a no-op, so set the clock separately
  Wire.setClock(hz);
  int who = readReg(0x75);
  if (who < 0) return -1;
  writeReg(0x6B, 0x80);          // device reset
  delay(100);
  bool ok = writeReg(0x6B, 0x01);  // wake, PLL with gyro X clock
  delay(10);
  ok &= writeReg(0x1A, 0x02);    // DLPF: ~94 Hz accel / 98 Hz gyro, 1 kHz internal
  ok &= writeReg(0x19, 4);       // sample rate = 1 kHz / (1 + 4) = 200 Hz
  ok &= writeReg(0x1B, 0x08);    // gyro +-500 dps
  ok &= writeReg(0x1C, 0x10);    // accel +-8 g (pothole hits can exceed 4 g)
  return ok ? who : -1;
}

bool imuRead(ImuSample& s) {
  Wire.beginTransmission(s_addr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint16_t)s_addr, (size_t)14, true) != 14) return false;
  int16_t r[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    r[i] = (int16_t)((hi << 8) | lo);
  }
  s.ax = r[0] / ACC_LSB_PER_G;
  s.ay = r[1] / ACC_LSB_PER_G;
  s.az = r[2] / ACC_LSB_PER_G;
  s.tempC = r[3] / 340.0f + 36.53f;
  s.gx = r[4] / GYR_LSB_PER_DPS;
  s.gy = r[5] / GYR_LSB_PER_DPS;
  s.gz = r[6] / GYR_LSB_PER_DPS;
  return true;
}
