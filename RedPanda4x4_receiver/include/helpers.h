#pragma once
#include <Arduino.h>
#include <esp_arduino_version.h>

// =======================================================
//              Integer helpers (shared)
// =======================================================

inline int clampInt(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

inline int mapInt(int x, int inMin, int inMax, int outMin, int outMax) {
  long num = (long)(x - inMin) * (outMax - outMin);
  long den = (inMax - inMin);
  return (int)(outMin + num / den);
}

inline int applyDeadzone(int v, int dz) {
  if (abs(v) < dz) return 0;
  return (v > 0) ? (v - dz) : (v + dz);
}

// =======================================================
//          LEDC write — Core-2 / Core-3 compat
// =======================================================

inline void ledcWriteCompat(int ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(ch, duty);
#else
  ledcWrite(ch, duty);
#endif
}

// =======================================================
//      Servo angle → duty (shared by servo modules)
// =======================================================

static const int SERVO_MIN_US = 650;
static const int SERVO_MAX_US = 2350;
static const int SERVO_FREQ   = 50;
static const int SERVO_RES    = 16;

inline uint32_t servoDutyFromUs(int pulseUs) {
  const int periodUs = 20000;
  pulseUs = clampInt(pulseUs, SERVO_MIN_US, SERVO_MAX_US);
  const uint32_t maxDuty = (1UL << SERVO_RES) - 1;
  return (uint32_t)((uint64_t)pulseUs * maxDuty / periodUs);
}

inline void servoWriteAngle(int ch, int angleDeg) {
  int pulseUs = mapInt(angleDeg, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  ledcWriteCompat(ch, servoDutyFromUs(pulseUs));
}
