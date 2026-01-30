#include <Arduino.h>
#include <WiFi.h>
#include "control_msg.h"
#include "mpu6500_reader.h"
#include "espnow_sender.h"

// MAC del receiver (ESP32 con TB6612)
static const uint8_t RX_MAC[6] = { 0xE0,0x8C,0xFE,0x2E,0x96,0x7C };

// Se AD0 è HIGH e il sensore risponde a 0x69, metti 0x69
static const uint8_t MPU_ADDR = 0x68;

static Mpu6500Reader mpu;
static EspNowSender nowTx;

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.print("TX STA MAC: ");
  Serial.println(WiFi.macAddress());

  mpu.begin(MPU_ADDR, 21, 22, 400000);

  if (!nowTx.begin(RX_MAC)) {
    Serial.println("ESP-NOW init/peer failed");
    while (true) delay(1000);
  }
}

void loop() {
  ControlMsg msg{0,0,0};

  if (!mpu.readAccel(msg)) {
    msg = {0,0,0};
    // Serial.println("MPU read fail -> sending zeros");
  } else {
    // Serial.printf("TX Axyz: %d %d %d\n", msg.ax, msg.ay, msg.az);
  }

  nowTx.send(RX_MAC, msg);
  delay(50);
}
