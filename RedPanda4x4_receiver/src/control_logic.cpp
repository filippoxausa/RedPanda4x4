#include "control_logic.h"
#include "helpers.h"
#include <Arduino.h>

// =======================================================
//          Quick-fix switches (calibrazione)
// =======================================================
static const bool INVERT_THROTTLE_AXIS = true;
static const bool INVERT_STEER_AXIS    = false;

// =======================================================
//                   Costanti
// =======================================================
static const int AX_AY_MAX    = 16200;
static const int DEADZONE     = 300;
static const int THR_SNAP_PWM = 35;

// =======================================================
//                  Mixing manuale
// =======================================================

MotorCmd accelToMotorsManual(const ControlMsg &a) {
  int steer = clampInt((int)a.ax, -AX_AY_MAX, AX_AY_MAX);
  int thr   = clampInt((int)a.ay, -AX_AY_MAX, AX_AY_MAX);

  if (INVERT_STEER_AXIS)    steer = -steer;
  if (INVERT_THROTTLE_AXIS) thr   = -thr;

  steer = applyDeadzone(steer, DEADZONE);
  thr   = applyDeadzone(thr,   DEADZONE);

  int steerPWM = mapInt(steer, -AX_AY_MAX, AX_AY_MAX, -255, 255);
  int thrPWM   = mapInt(thr,   -AX_AY_MAX, AX_AY_MAX, -255, 255);

  if (abs(thrPWM) < THR_SNAP_PWM) thrPWM = 0;

  int left = 0, right = 0;

  // Spin sul posto: throttle ~ 0 e steer forte
  const bool wantSpin = (thrPWM == 0) && (abs(steerPWM) > 60);

  if (wantSpin) {
    int spin = clampInt(steerPWM / 2, -255, 255);
    left  = spin;
    right = -spin;
  } else {
    int steerScaled = (steerPWM * abs(thrPWM)) / 255;
    left  = clampInt(thrPWM + steerScaled, -255, 255);
    right = clampInt(thrPWM - steerScaled, -255, 255);
  }

  return MotorCmd{ (int16_t)left, (int16_t)right };
}
