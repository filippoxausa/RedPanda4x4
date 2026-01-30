#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include "control_msg.h"

class EspNowSender {
public:
  bool begin(const uint8_t peerMac[6]);
  bool send(const uint8_t peerMac[6], const ControlMsg &msg);

private:
  static void onSent(const uint8_t *mac, esp_now_send_status_t status);
};
