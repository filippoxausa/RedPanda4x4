#pragma once
#include <Arduino.h>

class Mpu6500Reader {
public:
  /// Initialises the MPU (wake, config registers). Returns true on success.
  bool begin(uint8_t mpuAddr = 0x68, int sda = 21, int scl = 22, uint32_t clk = 400000);

  /// Calibrate offsets — keep the board still on a flat surface.
  void calibrate(uint32_t ms = 2000);

  /// Read raw accel+gyro, apply complementary filter. Returns true if read ok.
  bool updateAngles(float dt);

  /// Current estimated pitch (degrees, forward/backward).
  float pitchDeg() const { return pitchDeg_; }
  /// Current estimated roll  (degrees, left/right).
  float rollDeg()  const { return rollDeg_; }

private:
  uint8_t addr_ = 0x68;

  // Registers
  static constexpr uint8_t REG_WHO_AM_I     = 0x75;
  static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
  static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
  static constexpr uint8_t REG_CONFIG       = 0x1A;
  static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
  static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
  static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

  // Sensitivity (accel +/-2 g, gyro +/-250 dps)
  static constexpr float ACC_SENS_2G   = 16384.0f;
  static constexpr float GYRO_SENS_250 = 131.0f;

  // Complementary filter
  static constexpr float RAD2DEG = 57.2957795f;
  static constexpr float ALPHA   = 0.98f;

  // Estimated angles
  float pitchDeg_ = 0.0f;
  float rollDeg_  = 0.0f;

  // Calibration offsets
  float gxOff_ = 0, gyOff_ = 0, gzOff_ = 0;
  float axOff_ = 0, ayOff_ = 0, azOff_ = 0;
  float azSign_ = 1.0f;

  bool  write8(uint8_t reg, uint8_t val);
  bool  readBytes(uint8_t startReg, uint8_t *buf, size_t len);
  bool  readAccelGyroRaw(int16_t &ax, int16_t &ay, int16_t &az,
                         int16_t &gx, int16_t &gy, int16_t &gz);
};
