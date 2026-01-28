#include <Arduino.h>
#include <WiFi.h>
#include "control_msg.h"
#include "espnow_receiver.h"
#include "control_logic.h"
#include "motor_tb6612.h"

static EspNowReceiver nowRx;

void setup() {
  Serial.begin(115200);
  delay(200);

  motorsInit();

  Serial.print("RX STA MAC: ");
  Serial.println(WiFi.macAddress());

  if (!nowRx.begin()) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }
}

void loop() {
  ControlMsg a{};
  uint32_t ageMs = 0;
  nowRx.getLatest(a, ageMs);

  // Fail-safe: se non arrivano dati recenti, ferma
  if (ageMs > 300) {
    motorsStop();
  } else {
    MotorCmd cmd = accelToMotors(a);
    motorsSet(cmd.left, cmd.right);
  }

  static uint32_t t0 = 0;
  if (millis() - t0 > 200) {
    t0 = millis();
    // Serial.printf("Axyz=%d %d %d age=%lu ms\n", a.ax, a.ay, a.az, (unsigned long)ageMs);
  }

  delay(5);
}
