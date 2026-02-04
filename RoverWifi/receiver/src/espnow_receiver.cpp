#include "espnow_receiver.h"
#include <WiFi.h>
#include <esp_now.h>
#include <string.h>

volatile int16_t  EspNowReceiver::s_ax = 0;
volatile int16_t  EspNowReceiver::s_ay = 0;
volatile int16_t  EspNowReceiver::s_az = 0;
volatile uint32_t EspNowReceiver::s_lastRxMs = 0;

void EspNowReceiver::onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
  if (len != (int)sizeof(ControlMsg)) return;

  ControlMsg msg;
  memcpy(&msg, data, sizeof(msg));

  s_ax = msg.ax;
  s_ay = msg.ay;
  s_az = msg.az;
  s_lastRxMs = millis();
}

bool EspNowReceiver::begin() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(EspNowReceiver::onRecv);

  s_lastRxMs = millis();
  return true;
}

void EspNowReceiver::getLatest(ControlMsg &out, uint32_t &ageMs) const {
  out.ax = s_ax;
  out.ay = s_ay;
  out.az = s_az;

  uint32_t now = millis();
  uint32_t last = s_lastRxMs;
  ageMs = now - last;
}