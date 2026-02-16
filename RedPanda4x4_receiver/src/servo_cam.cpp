#include "servo_cam.h"
#include "helpers.h"

// =======================================================
//                     Pin config
// =======================================================
static const int PIN_SERVO_CAM = 32;
static const int CH_SERVO_CAM  = 3;

static const int CAM_MIN_ANG = 35;
static const int CAM_MAX_ANG = 145;

static const int      CAM_STEP_DEG = 1;
static const uint32_t CAM_STEP_MS  = 35;

// =======================================================
//                       State
// =======================================================
static int      camCur    = 90;
static int      camTgt    = 90;
static uint32_t lastCamMs = 0;

static void writeAngle(int ang) {
  ang = clampInt(ang, CAM_MIN_ANG, CAM_MAX_ANG);
  servoWriteAngle(CH_SERVO_CAM, ang);
}

// =======================================================
//                    Public API
// =======================================================

void servoCamInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(PIN_SERVO_CAM, SERVO_FREQ, SERVO_RES, CH_SERVO_CAM);
#else
  ledcSetup(CH_SERVO_CAM, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(PIN_SERVO_CAM, CH_SERVO_CAM);
#endif

  camCur = 90;
  camTgt = 90;
  writeAngle(camCur);
}

void servoCamSetTarget(int angleDeg) {
  camTgt = clampInt(angleDeg, CAM_MIN_ANG, CAM_MAX_ANG);
}

void servoCamUpdate() {
  uint32_t now = millis();
  if (now - lastCamMs < CAM_STEP_MS) return;
  lastCamMs = now;

  if      (camCur < camTgt) camCur += CAM_STEP_DEG;
  else if (camCur > camTgt) camCur -= CAM_STEP_DEG;

  writeAngle(camCur);
}

int servoCamCurrent() { return camCur; }
