#pragma once
#include <stdint.h>

class MPU6500 {
public:
  bool begin(int sdaPin = 21, int sclPin = 22, uint32_t clockHz = 400000);
  bool readAccelY(int16_t &ay);

private:
  static const uint8_t MPU_ADDR = 0x68;      // se AD0 alto -> 0x69
  static const uint8_t REG_PWR_MGMT_1 = 0x6B;
  static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

  bool readBytes(uint8_t startReg, uint8_t *buf, size_t len);
  void write8(uint8_t reg, uint8_t val);
};
