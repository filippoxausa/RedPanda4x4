#include "motor_tb6612.h"
#include "helpers.h"

// =======================================================
//           Quick-fix switches (calibrazione)
// =======================================================
static const bool INVERT_LEFT_MOTOR  = false;
static const bool INVERT_RIGHT_MOTOR = true;

// =======================================================
//                     Pin config
// =======================================================
static const int PIN_AIN1 = 14;
static const int PIN_AIN2 = 27;
static const int PIN_PWMA = 26;

static const int PIN_BIN1 = 16;
static const int PIN_BIN2 = 17;
static const int PIN_PWMB = 4;

static const int PIN_STBY = 25;

static const int PWM_FREQ = 20000;
static const int PWM_RES  = 8;
static const int CH_A = 0;   // left
static const int CH_B = 1;   // right

// =======================================================
//                    Low-level
// =======================================================
static void motorEnable(bool en) {
  digitalWrite(PIN_STBY, en ? HIGH : LOW);
}

static void driveMotor(int in1, int in2, int pwmCh, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    ledcWriteCompat(pwmCh, (uint32_t)speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    ledcWriteCompat(pwmCh, (uint32_t)(-speed));
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    ledcWriteCompat(pwmCh, 0);
  }
}

static void driveLeft(int speed) {
  if (INVERT_LEFT_MOTOR) speed = -speed;
  driveMotor(PIN_AIN1, PIN_AIN2, CH_A, speed);
}

static void driveRight(int speed) {
  if (INVERT_RIGHT_MOTOR) speed = -speed;
  driveMotor(PIN_BIN1, PIN_BIN2, CH_B, speed);
}

// =======================================================
//              Rampa anti-slip
// =======================================================
static int16_t g_curL = 0, g_curR = 0;
static const int16_t RAMP_STEP = 14;
static const int16_t DIFF_MAX  = 230;

static int16_t stepToward(int16_t cur, int16_t tgt, int16_t step) {
  if (tgt > cur) return (int16_t)min<int>(tgt, cur + step);
  if (tgt < cur) return (int16_t)max<int>(tgt, cur - step);
  return cur;
}

// =======================================================
//                   Public API
// =======================================================

void motorsInit() {
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(PIN_PWMA, PWM_FREQ, PWM_RES, CH_A);
  ledcAttachChannel(PIN_PWMB, PWM_FREQ, PWM_RES, CH_B);
#else
  ledcSetup(CH_A, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_PWMA, CH_A);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_PWMB, CH_B);
#endif

  motorEnable(true);
  driveLeft(0);
  driveRight(0);
}

void motorsRaw(int16_t left, int16_t right) {
  driveLeft(left);
  driveRight(right);
}

void motorsSetSmooth(int16_t targetL, int16_t targetR) {
  targetL = constrain(targetL, -255, 255);
  targetR = constrain(targetR, -255, 255);

  // Limitazione differenziale
  int diff = targetL - targetR;
  if (diff > DIFF_MAX) {
    int mid = (targetL + targetR) / 2;
    targetL = mid + DIFF_MAX / 2;
    targetR = mid - DIFF_MAX / 2;
  } else if (diff < -DIFF_MAX) {
    int mid = (targetL + targetR) / 2;
    targetL = mid - DIFF_MAX / 2;
    targetR = mid + DIFF_MAX / 2;
  }

  g_curL = stepToward(g_curL, targetL, RAMP_STEP);
  g_curR = stepToward(g_curR, targetR, RAMP_STEP);
  motorsRaw(g_curL, g_curR);
}

void motorsStop() {
  motorsSetSmooth(0, 0);
}
