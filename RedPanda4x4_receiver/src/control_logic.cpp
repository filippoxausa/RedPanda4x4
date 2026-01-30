#include "control_logic.h"
#include <Arduino.h>

static inline int abs16(int16_t v) { return v < 0 ? -v : v; }

// MotorCmd accelToMotors(const ControlMsg &a) {
//   // Deadband: evita tremolii
//   const int16_t AX_DEADBAND = 3000;
//   const int16_t AY_DEADBAND = 3000;

//   // “Full tilt” -> comando massimo
//   const int16_t AX_FULL = 16000;
//   const int16_t AY_FULL = 16000;

//   // Spin-in-place se y ~ 0 e x grande
//   const int16_t SPIN_Y_MAX = 4000;
//   const int16_t SPIN_X_MIN = 12000;

//   int16_t ax = (abs16(a.ax) < AX_DEADBAND) ? 0 : a.ax;
  // int16_t ay = (abs16(a.ay) < AY_DEADBAND) ? 0 : a.ay;

//   // Se ti va “al contrario”, inverti ay: ay = -ay;
//   int throttle = (ay * 255) / AY_FULL;  // -255..255
//   int turn     = (ax * 255) / AX_FULL;  // -255..255
//   throttle = constrain(throttle, -255, 255);
//   turn     = constrain(turn, -255, 255);

//   // Modalità spin
//   if (abs16(a.ay) < SPIN_Y_MAX && abs16(a.ax) > SPIN_X_MIN) {
//     throttle = 0;
//   }

//   // Arcade mixing: L = throttle + turn, R = throttle - turn
//   int left  = throttle + turn;
//   int right = throttle - turn;

//   // Saturazione “morbida” (mantiene rapporto)
//   int m = max(abs(left), abs(right));
//   if (m > 255) {
//     left  = (left  * 255) / m;
//     right = (right * 255) / m;
//   }

//   return MotorCmd{ (int16_t)left, (int16_t)right };
// }


MotorCmd accelToMotors(const ControlMsg &a) {
  const int16_t DEADBAND = 3000;

  int16_t x = (abs16(a.ax) < DEADBAND) ? 0 : a.ax;
  int16_t y = (abs16(a.ay) < DEADBAND) ? 0 : a.ay;

  int throttle = 0;
  int turn = 0;

  if (y != 0) {
    throttle = (y * 255) / 16000;
    throttle = constrain(throttle, -255, 255);
    turn = 0;
  } else if (x != 0) {
    turn = (x * 255) / 16000;
    turn = constrain(turn, -255, 255);
    throttle = 0;
  }

  int left  = throttle + turn;
  int right = throttle - turn; // mixing arcade: left=x+y, right=y-x (equivalente)

  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  return MotorCmd{ (int16_t)left, (int16_t)right };
}
