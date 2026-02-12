#include "mpu6500_reader.h"
#include <Wire.h>
#include <math.h>

bool Mpu6500Reader::write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool Mpu6500Reader::readBytes(uint8_t startReg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr_);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr_, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool Mpu6500Reader::readAccelGyroRaw(int16_t &ax, int16_t &ay, int16_t &az,
                                      int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t buf[14];
  if (!readBytes(REG_ACCEL_XOUT_H, buf, 14)) return false;

  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
  // buf[6..7] = temperature, skip
  gx = (int16_t)((buf[8]  << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);
  return true;
}

bool Mpu6500Reader::begin(uint8_t mpuAddr, int sda, int scl, uint32_t clk) {
  addr_ = mpuAddr;

  Wire.begin(sda, scl);
  Wire.setClock(clk);

  if (!write8(REG_PWR_MGMT_1, 0x00)) return false;  // wake
  delay(50);

  write8(REG_CONFIG, 0x03);       // DLPF 44 Hz
  write8(REG_SMPLRT_DIV, 0x04);   // sample rate divider

  if (!write8(REG_GYRO_CONFIG, 0x00)) return false;   // +/-250 dps
  delay(10);

  if (!write8(REG_ACCEL_CONFIG, 0x00)) return false;  // +/-2 g
  delay(10);

  uint8_t who = 0;
  if (!readBytes(REG_WHO_AM_I, &who, 1)) return false;
  Serial.printf("[MPU] WHO_AM_I=0x%02X\n", who);
  return true;
}

void Mpu6500Reader::calibrate(uint32_t ms) {
  Serial.println("[MPU] Calibrating... keep STILL on a flat surface");

  uint32_t t0 = millis();
  uint32_t n  = 0;

  double sumAx = 0, sumAy = 0, sumAz = 0;
  double sumGx = 0, sumGy = 0, sumGz = 0;

  while (millis() - t0 < ms) {
    int16_t rax, ray, raz, rgx, rgy, rgz;
    if (readAccelGyroRaw(rax, ray, raz, rgx, rgy, rgz)) {
      float axg = (float)rax / ACC_SENS_2G;
      float ayg = (float)ray / ACC_SENS_2G;
      float azg = (float)raz / ACC_SENS_2G;

      float gxd = (float)rgx / GYRO_SENS_250;
      float gyd = (float)rgy / GYRO_SENS_250;
      float gzd = (float)rgz / GYRO_SENS_250;

      sumAx += axg; sumAy += ayg; sumAz += azg;
      sumGx += gxd; sumGy += gyd; sumGz += gzd;
      n++;
    }
    delay(5);
  }

  if (n == 0) return;

  float meanAx = (float)(sumAx / n);
  float meanAy = (float)(sumAy / n);
  float meanAz = (float)(sumAz / n);

  gxOff_ = (float)(sumGx / n);
  gyOff_ = (float)(sumGy / n);
  gzOff_ = (float)(sumGz / n);

  azSign_ = (meanAz >= 0.0f) ? 1.0f : -1.0f;

  axOff_ = meanAx;
  ayOff_ = meanAy;
  azOff_ = meanAz - azSign_ * 1.0f;

  float ax0 = meanAx - axOff_;
  float ay0 = meanAy - ayOff_;
  float az0 = meanAz - azOff_;

  float pitchAcc0 = atan2f(ay0, sqrtf(ax0 * ax0 + az0 * az0)) * RAD2DEG;
  float rollAcc0  = atan2f(-ax0, az0) * RAD2DEG;

  pitchDeg_ = pitchAcc0;
  rollDeg_  = rollAcc0;

  Serial.printf("[MPU] Cal done. mean(g): ax=%.3f ay=%.3f az=%.3f | azSign=%.0f\n",
                meanAx, meanAy, meanAz, azSign_);
}

bool Mpu6500Reader::updateAngles(float dt) {
  int16_t rax = 0, ray = 0, raz = 0, rgx = 0, rgy = 0, rgz = 0;
  if (!readAccelGyroRaw(rax, ray, raz, rgx, rgy, rgz)) return false;

  float axg = (float)rax / ACC_SENS_2G - axOff_;
  float ayg = (float)ray / ACC_SENS_2G - ayOff_;
  float azg = (float)raz / ACC_SENS_2G - azOff_;

  float gxd = (float)rgx / GYRO_SENS_250 - gxOff_;
  float gyd = (float)rgy / GYRO_SENS_250 - gyOff_;

  float pitchAcc = atan2f(ayg, sqrtf(axg * axg + azg * azg)) * RAD2DEG;
  float rollAcc  = atan2f(-axg, azg) * RAD2DEG;

  float pitchGyro = pitchDeg_ + (gxd * dt);
  float rollGyro  = rollDeg_  + (gyd * dt);

  pitchDeg_ = ALPHA * pitchGyro + (1.0f - ALPHA) * pitchAcc;
  rollDeg_  = ALPHA * rollGyro  + (1.0f - ALPHA) * rollAcc;

  return true;
}
