#include "joystick.h"

// =======================================================
//                   DriveJoystick
// =======================================================

void DriveJoystick::begin(int pinVRX, int pinVRY, int pinSW, int deadzone) {
  pinX_ = pinVRX;
  pinY_ = pinVRY;
  pinSW_ = pinSW;
  deadzone_ = deadzone;
  pinMode(pinSW_, INPUT_PULLUP);
}

void DriveJoystick::calibrateCenter(int samples) {
  long sx = 0, sy = 0;
  for (int i = 0; i < samples; i++) {
    sx += analogRead(pinX_);
    sy += analogRead(pinY_);
    delay(2);
  }
  xCenter_ = (int)(sx / samples);
  yCenter_ = (int)(sy / samples);
  Serial.printf("[JOY] center calibrated: x=%d y=%d\n", xCenter_, yCenter_);
}

void DriveJoystick::update() {
  int xRaw = analogRead(pinX_);
  int yRaw = analogRead(pinY_);
  btnPressed_ = (digitalRead(pinSW_) == LOW);

  // Asymmetric mapping relative to calibrated center
  int xRel = xRaw - xCenter_;
  int yRel = yRaw - yCenter_;

  if (xRel >= 0) {
    int maxR = 4095 - xCenter_;
    xMapped_ = (maxR > 0) ? (int)((long)xRel * OUTPUT_MAX / maxR) : 0;
  } else {
    xMapped_ = (xCenter_ > 0) ? (int)((long)xRel * OUTPUT_MAX / xCenter_) : 0;
  }

  if (yRel >= 0) {
    int maxR = 4095 - yCenter_;
    yMapped_ = (maxR > 0) ? (int)((long)yRel * OUTPUT_MAX / maxR) : 0;
  } else {
    yMapped_ = (yCenter_ > 0) ? (int)((long)yRel * OUTPUT_MAX / yCenter_) : 0;
  }

  // Deadzone with rescaling + clamp
  xMapped_ = applyDeadzone(xMapped_, deadzone_);
  yMapped_ = applyDeadzone(yMapped_, deadzone_);
  xMapped_ = clampInt(xMapped_, -OUTPUT_MAX, OUTPUT_MAX);
  yMapped_ = clampInt(yMapped_, -OUTPUT_MAX, OUTPUT_MAX);
}

// =======================================================
//                    CamJoystick
// =======================================================

void CamJoystick::begin(int pinVRX, int pinSW, int deadzone) {
  pinX_ = pinVRX;
  pinSW_ = pinSW;
  deadzone_ = deadzone;
  pinMode(pinSW_, INPUT_PULLUP);
}

void CamJoystick::update() {
  int rawX = analogRead(pinX_);
  xMapped_ = map(rawX, 0, 4095, -OUTPUT_MAX, OUTPUT_MAX);
  xMapped_ = applyDeadzone(xMapped_, deadzone_);
  xMapped_ = clampInt(xMapped_, -OUTPUT_MAX, OUTPUT_MAX);

  centerPressed_ = (digitalRead(pinSW_) == LOW);
  if (centerPressed_) xMapped_ = 0;
}
