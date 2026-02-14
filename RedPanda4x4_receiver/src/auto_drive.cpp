#include "auto_drive.h"
#include "motor_tb6612.h"
#include "servo_ultrasonic.h"
#include "helpers.h"

// =======================================================
//                   Soglie e tempi
// =======================================================
static const uint16_t AUTO_FRONT_TRIG_CM = 15;   // ostacolo frontale (cm)
static const uint16_t AUTO_SIDE_FREE_CM  = 25;   // lato "libero" (cm)

static const int16_t  AUTO_FWD_SPEED  = 160;
static const int16_t  AUTO_BACK_SPEED = 150;

// Curva ad arco: esterna veloce, interna lenta
static const int16_t  AUTO_TURN_OUTER = 200;
static const int16_t  AUTO_TURN_INNER = 60;
static const uint32_t AUTO_TURN_MS    = 1100;

static const uint32_t AUTO_STOP_MS   = 100;
static const uint32_t AUTO_BACK_MS   = 1000;

static const uint32_t SCAN_SERVO_SETTLE_MS = 450;

// =======================================================
//                     State
// =======================================================
enum AutoState : uint8_t { A_FWD = 0, A_STOP, A_SCAN, A_BACK, A_TURN };

static AutoState autoSt      = A_FWD;
static uint32_t  autoTs      = 0;
static int8_t    autoTurnDir = +1;     // +1 destra, -1 sinistra

static uint16_t dL = 999, dR = 999;
static uint8_t  scanPhase = 0;

static bool g_autoObstacle = false;
static bool g_isReverseNow = false;

// =======================================================
//                    Public API
// =======================================================

void autoReset() {
  autoSt = A_FWD;
  autoTs = millis();
  autoTurnDir = +1;
  scanPhase   = 0;
  servoUsWrite(servoUsCenterAngle());
  dL = dR = 999;
  g_autoObstacle = false;
  g_isReverseNow = false;
}

bool autoIsObstacle()  { return g_autoObstacle; }
bool autoIsReversing() { return g_isReverseNow; }

void runAuto(bool backObs) {
  const uint32_t now = millis();
  g_isReverseNow = false;

  switch (autoSt) {

    // ---- avanti dritto, controlla davanti ----
    case A_FWD: {
      servoUsWrite(servoUsCenterAngle());
      motorsSetSmooth(AUTO_FWD_SPEED, AUTO_FWD_SPEED);

      uint16_t front = clampDist(hcsr04ReadCm());
      g_autoObstacle = (front <= AUTO_FRONT_TRIG_CM);

      if (front <= AUTO_FRONT_TRIG_CM) {
        motorsStop();
        autoSt = A_STOP;
        autoTs = now;
      }
    } break;

    // ---- breve pausa, poi scansione ----
    case A_STOP: {
      motorsStop();
      servoUsWrite(servoUsCenterAngle());
      g_autoObstacle = true;

      if (now - autoTs >= AUTO_STOP_MS) {
        scanPhase = 0;
        dL = dR = 999;
        autoSt = A_SCAN;
        autoTs = now;
      }
    } break;

    // ---- scansione sinistra + destra, poi decisione ----
    case A_SCAN: {
      motorsStop();
      g_autoObstacle = true;

      switch (scanPhase) {
        case 0:
          servoUsWrite(servoUsMaxAngle());
          scanPhase = 1;
          autoTs = now;
          break;

        case 1:
          if (now - autoTs >= SCAN_SERVO_SETTLE_MS) {
            dL = clampDist(hcsr04ReadCm());
            servoUsWrite(servoUsMinAngle());
            scanPhase = 2;
            autoTs = now;
          }
          break;

        case 2:
          if (now - autoTs >= SCAN_SERVO_SETTLE_MS) {
            dR = clampDist(hcsr04ReadCm());
            servoUsWrite(servoUsCenterAngle());
            scanPhase = 3;
            autoTs = now;
          }
          break;

        case 3: {
          bool leftFree  = (dL >= AUTO_SIDE_FREE_CM);
          bool rightFree = (dR >= AUTO_SIDE_FREE_CM);

          if (!leftFree && !rightFree) {
            autoSt = A_BACK;
            autoTs = now;
          } else {
            if      (leftFree && !rightFree)  autoTurnDir = -1;
            else if (!leftFree && rightFree)  autoTurnDir = +1;
            else autoTurnDir = (dL >= dR) ? -1 : +1;

            autoSt = A_TURN;
            autoTs = now;
          }
        } break;
      }
    } break;

    // ---- retromarcia, poi ri-scan ----
    case A_BACK: {
      g_autoObstacle = true;
      g_isReverseNow = true;

      if (backObs) {
        motorsStop();
        autoTurnDir = -autoTurnDir;
        autoSt = A_TURN;
        autoTs = now;
        break;
      }

      motorsSetSmooth(-AUTO_BACK_SPEED, -AUTO_BACK_SPEED);

      if (now - autoTs >= AUTO_BACK_MS) {
        motorsStop();
        autoSt = A_STOP;
        autoTs = now;
      }
    } break;

    // ---- curva ad arco ----
    case A_TURN: {
      g_autoObstacle = true;
      servoUsWrite(servoUsCenterAngle());

      int16_t Lm, Rm;
      if (autoTurnDir > 0) {          // curva a destra
        Lm = AUTO_TURN_OUTER;
        Rm = AUTO_TURN_INNER;
      } else {                         // curva a sinistra
        Lm = AUTO_TURN_INNER;
        Rm = AUTO_TURN_OUTER;
      }
      motorsSetSmooth(Lm, Rm);

      if (now - autoTs >= AUTO_TURN_MS) {
        motorsStop();
        autoSt = A_FWD;
        autoTs = now;
      }
    } break;
  }
}
