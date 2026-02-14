#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>

#include "control_msg.h"
#include "espnow_receiver.h"
#include "control_logic.h"
#include "motor_tb6612.h"
#include "buzzer.h"
#include "ir_sensor.h"
#include "servo_ultrasonic.h"
#include "servo_cam.h"
#include "display_oled.h"
#include "auto_drive.h"
#include "helpers.h"

// =======================================================
//                    Globals
// =======================================================
static EspNowReceiver rx;
static const uint32_t RX_TIMEOUT_MS = 250;

// throttle del display per non saturare I2C
static uint32_t lastDispMs = 0;

// costanti condivise con control_logic per la camera
static const int AX_AY_MAX = 16200;

// INVERT_THROTTLE_AXIS ripetuto qui per il blocco retro IR
static const bool INVERT_THROTTLE_AXIS = true;
static const int  DEADZONE = 300;

// =======================================================
//                   Self-test iniziale
// =======================================================
static void selfTest() {
  Serial.println("\n--- SELF TEST ---");

  Serial.println("Servo US sweep...");
  for (int a = servoUsMinAngle(); a <= servoUsMaxAngle(); a += 5) {
    servoUsWrite(a); delay(20);
  }
  for (int a = servoUsMaxAngle(); a >= servoUsMinAngle(); a -= 5) {
    servoUsWrite(a); delay(20);
  }
  servoUsWrite(servoUsCenterAngle());

  Serial.println("Motors: LEFT forward...");
  motorsSetSmooth(140, 0); delay(700);
  motorsStop(); delay(300);

  Serial.println("Motors: RIGHT forward...");
  motorsSetSmooth(0, 140); delay(700);
  motorsStop(); delay(300);

  Serial.println("Motors: BOTH forward...");
  motorsSetSmooth(140, 140); delay(700);
  motorsStop(); delay(300);

  Serial.println("--- END SELF TEST ---\n");
}

// =======================================================
//                       setup()
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  motorsInit();
  servoUsInit();
  hcsr04Init();
  servoCamInit();
  irInit();
  buzzerInit();

  // I2C per display OLED
  Wire.begin(21, 22);

  if (displayInit()) {
    drawProjectName();
  }

  Serial.print("RX STA MAC: ");
  Serial.println(WiFi.macAddress());

  if (!rx.begin()) {
    Serial.println("ESP-NOW init FAILED!");
    motorsStop();
    while (true) delay(1000);
  }
  Serial.println("ESP-NOW init OK");

  motorsStop();
  autoReset();
  selfTest();
}

// =======================================================
//                       loop()
// =======================================================
void loop() {
  ControlMsg msg{};
  uint32_t ageMs = 0;
  rx.getLatest(msg, ageMs);

  const uint32_t now = millis();
  const bool backObs = irBackObstacle();

  // ---- FAILSAFE: in manuale, stop se radio persa ----
  if (!msg.autoMode && ageMs > RX_TIMEOUT_MS) {
    motorsStop();
    servoUsWrite(servoUsCenterAngle());
    servoCamUpdate();
    buzzerUpdate(false);

    if (now - lastDispMs > 120) {
      lastDispMs = now;
      drawProjectName();
    }

    delay(5);
    return;
  }

  bool wantReverseBeep = false;

  // ---- GUIDA AUTONOMA ----
  if (msg.autoMode) {
    runAuto(backObs);
    servoCamSetTarget(90);
    wantReverseBeep = autoIsReversing();

    if (now - lastDispMs > 120) {
      lastDispMs = now;
      if (autoIsObstacle()) drawSad();
      else                  drawHappy();
    }

  // ---- GUIDA MANUALE (joystick o tilt) ----
  } else {
    autoReset();

    MotorCmd m = accelToMotorsManual(msg);

    // Blocco retro se IR vede ostacolo
    const int ayEff = INVERT_THROTTLE_AXIS ? -msg.ay : msg.ay;
    const bool wantReverse = (ayEff < -DEADZONE);
    wantReverseBeep = wantReverse;

    if (wantReverse && backObs) {
      if (m.left  < 0) m.left  = 0;
      if (m.right < 0) m.right = 0;
      wantReverseBeep = false;
    }

    motorsSetSmooth(m.left, m.right);

    // Servo US al centro in manuale
    servoUsWrite(servoUsCenterAngle());

    // Camera da az
    int az = clampInt((int)msg.az, -AX_AY_MAX, AX_AY_MAX);
    int camAng = mapInt(az, -AX_AY_MAX, AX_AY_MAX, 0, 180);
    servoCamSetTarget(camAng);

    // Display: nome progetto
    if (now - lastDispMs > 250) {
      lastDispMs = now;
      drawProjectName();
    }
  }

  // ---- Buzzer retromarcia ----
  buzzerUpdate(wantReverseBeep);

  // ---- Camera smoothing ----
  servoCamUpdate();

  delay(5);
}
