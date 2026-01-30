#pragma once
#include <Arduino.h>
#include "control_msg.h"

class Mpu6500Reader {
public:
  bool begin(uint8_t mpuAddr = 0x68, int sda = 21, int scl = 22, uint32_t clk = 400000);
  bool readAccel(ControlMsg &out);

private:
  uint8_t addr_ = 0x68;

  static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
  static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

  void write8(uint8_t reg, uint8_t val);
  bool readBytes(uint8_t startReg, uint8_t *buf, size_t len);
};
