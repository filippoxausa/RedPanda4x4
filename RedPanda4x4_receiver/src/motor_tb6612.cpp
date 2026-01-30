#include "motor_tb6612.h"

static const int PIN_AIN1 = 14;
static const int PIN_AIN2 = 27;
static const int PIN_PWMA = 26;

static const int PIN_BIN1 = 16;
static const int PIN_BIN2 = 17;
static const int PIN_PWMB = 4;

static const int PIN_STBY = 25;

static const int PWM_FREQ = 20000;
static const int PWM_RES  = 8;
static const int CH_A = 0; // left
static const int CH_B = 1; // right

static void motorEnable(bool en) {
  digitalWrite(PIN_STBY, en ? HIGH : LOW);
}

static void driveMotor(int in1, int in2, int pwmCh, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmCh, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmCh, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmCh, 0);
  }
}

static void driveLeft(int speed)  { driveMotor(PIN_AIN1, PIN_AIN2, CH_A, speed); }
static void driveRight(int speed) { driveMotor(PIN_BIN1, PIN_BIN2, CH_B, speed); }

void motorsInit() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  ledcSetup(CH_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWMA, CH_A);

  ledcSetup(CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWMB, CH_B);

  motorEnable(true);
  motorsStop();
}

void motorsSet(int16_t left, int16_t right) {
  driveLeft(left);
  driveRight(right);
}

void motorsStop() {
  motorsSet(0, 0);
}
