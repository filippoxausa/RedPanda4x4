#include "mpu6500_reader.h"
#include <Wire.h>

bool Mpu6500Reader::begin(uint8_t mpuAddr, int sda, int scl, uint32_t clk) {
  addr_ = mpuAddr;

  Wire.begin(sda, scl);
  Wire.setClock(clk);

  write8(REG_PWR_MGMT_1, 0x00); // wake
  delay(10);
  return true;
}

void Mpu6500Reader::write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

bool Mpu6500Reader::readBytes(uint8_t startReg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr_);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t n = Wire.requestFrom((int)addr_, (int)len);
  if (n != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool Mpu6500Reader::readAccel(ControlMsg &out) {
  uint8_t raw[6];
  if (!readBytes(REG_ACCEL_XOUT_H, raw, sizeof(raw))) return false;

  out.ax = (int16_t)((raw[0] << 8) | raw[1]);
  out.ay = (int16_t)((raw[2] << 8) | raw[3]);
  out.az = (int16_t)((raw[4] << 8) | raw[5]);
  return true;
}

// --- NUOVO: gyro Z raw (2 bytes) ---
bool Mpu6500Reader::readGyroZRaw(int16_t &gzRaw) {
  uint8_t raw[2];
  if (!readBytes(REG_GYRO_ZOUT_H, raw, sizeof(raw))) return false;
  gzRaw = (int16_t)((raw[0] << 8) | raw[1]);
  return true;
}
