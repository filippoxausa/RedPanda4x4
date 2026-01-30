#include "espnow_sender.h"
#include <WiFi.h>
#include <string.h>

void EspNowSender::onSent(const uint8_t *mac, esp_now_send_status_t status) {
  (void)mac;
  // Serial.print("ESP-NOW send: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

bool EspNowSender::begin(const uint8_t peerMac[6]) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_send_cb(EspNowSender::onSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, peerMac, 6);
  peer.channel = 0;
  peer.encrypt = false;

  return (esp_now_add_peer(&peer) == ESP_OK);
}

bool EspNowSender::send(const uint8_t peerMac[6], const ControlMsg &msg) {
  return (esp_now_send(peerMac, (const uint8_t*)&msg, sizeof(msg)) == ESP_OK);
}
