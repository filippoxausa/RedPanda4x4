#pragma once
#include <Arduino.h>

// =======================================================
//           Shared output range & helpers
// =======================================================
static const int OUTPUT_MAX = 16000;

inline int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/// Deadzone with rescaling: output still reaches +/-OUTPUT_MAX
inline int applyDeadzone(int v, int dz) {
  if (abs(v) <= dz) return 0;
  int sign = (v > 0) ? 1 : -1;
  return sign * (int)((long)(abs(v) - dz) * OUTPUT_MAX / (OUTPUT_MAX - dz));
}

// =======================================================
//         Drive joystick (2 axes + mode button)
// =======================================================
class DriveJoystick {
public:
  /// Configure pins. Call after analogReadResolution/analogSetAttenuation.
  void begin(int pinVRX, int pinVRY, int pinSW, int deadzone = 1300);

  /// Calibrate center position — don't touch the stick!
  void calibrateCenter(int samples = 200);

  /// Read raw inputs + compute mapped outputs.
  void update();

  /// Mapped values in [-OUTPUT_MAX, +OUTPUT_MAX]
  int x() const { return xMapped_; }
  int y() const { return yMapped_; }
  bool button() const { return btnPressed_; }

private:
  int pinX_ = -1, pinY_ = -1, pinSW_ = -1;
  int deadzone_ = 1300;
  int xCenter_ = 2048, yCenter_ = 2048;
  int xMapped_ = 0, yMapped_ = 0;
  bool btnPressed_ = false;
};

// =======================================================
//       Camera-pan joystick (1 axis + center button)
// =======================================================
class CamJoystick {
public:
  void begin(int pinVRX, int pinSW, int deadzone = 1300);
  void update();

  /// Mapped pan value in [-OUTPUT_MAX, +OUTPUT_MAX]
  int x() const { return xMapped_; }
  bool centerButton() const { return centerPressed_; }

private:
  int pinX_ = -1, pinSW_ = -1;
  int deadzone_ = 1300;
  int xMapped_ = 0;
  bool centerPressed_ = false;
};
