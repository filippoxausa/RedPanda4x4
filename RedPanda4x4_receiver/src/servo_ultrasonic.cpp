#include "servo_ultrasonic.h"
#include "helpers.h"

// =======================================================
//                     Pin config
// =======================================================
static const int PIN_SERVO_US = 33;
static const int CH_SERVO_US  = 2;

static const int PIN_TRIG = 18;
static const int PIN_ECHO = 19;

// Sweep angles
static const int US_CENTER  = 40;   // "straight ahead" angle
static const int US_MIN_ANG = 0;
static const int US_MAX_ANG = 80;

// =======================================================
//                     Servo US
// =======================================================

void servoUsInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(PIN_SERVO_US, SERVO_FREQ, SERVO_RES, CH_SERVO_US);
#else
  ledcSetup(CH_SERVO_US, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(PIN_SERVO_US, CH_SERVO_US);
#endif
  servoUsWrite(US_CENTER);
}

void servoUsWrite(int angleDeg) {
  angleDeg = clampInt(angleDeg, US_MIN_ANG, US_MAX_ANG);
  servoWriteAngle(CH_SERVO_US, angleDeg);
}

// =======================================================
//                      HC-SR04
// =======================================================

void hcsr04Init() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
}

uint16_t hcsr04ReadCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  uint32_t us = pulseIn(PIN_ECHO, HIGH, 25000);
  if (us == 0) return 999;
  return (uint16_t)(us / 58);
}

uint16_t clampDist(uint16_t d) {
  if (d == 0 || d > 400) return 999;
  return d;
}

// =======================================================
//                   Angle getters
// =======================================================
int servoUsCenterAngle() { return US_CENTER; }
int servoUsMinAngle()    { return US_MIN_ANG; }
int servoUsMaxAngle()    { return US_MAX_ANG; }
