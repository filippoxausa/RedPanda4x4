#include "espnow_receiver.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <string.h>

volatile bool     EspNowReceiver::s_auto = false;
volatile int16_t  EspNowReceiver::s_ax   = 0;
volatile int16_t  EspNowReceiver::s_ay   = 0;
volatile int16_t  EspNowReceiver::s_az   = 0;
volatile uint32_t EspNowReceiver::s_lastRxMs = 0;

// =======================================================
//           Callback (Core-2 / Core-3 compat)
// =======================================================

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void EspNowReceiver::onRecv(const esp_now_recv_info_t *info,
                            const uint8_t *data, int len) {
  (void)info;
#else
void EspNowReceiver::onRecv(const uint8_t *mac,
                            const uint8_t *data, int len) {
  (void)mac;
#endif
  if (len != (int)sizeof(ControlMsg)) return;

  ControlMsg msg;
  memcpy(&msg, data, sizeof(msg));

  s_auto = msg.autoMode;
  s_ax   = msg.ax;
  s_ay   = msg.ay;
  s_az   = msg.az;
  s_lastRxMs = millis();
}

// =======================================================
//                     begin / getLatest
// =======================================================

bool EspNowReceiver::begin() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  // Force channel 1 — must match the sender
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  Serial.println("[ESP-NOW] RX channel forced to 1");

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(EspNowReceiver::onRecv);

  s_lastRxMs = millis();
  return true;
}

void EspNowReceiver::getLatest(ControlMsg &out, uint32_t &ageMs) const {
  out.autoMode = s_auto;
  out.ax = s_ax;
  out.ay = s_ay;
  out.az = s_az;
  ageMs  = millis() - (uint32_t)s_lastRxMs;
}
