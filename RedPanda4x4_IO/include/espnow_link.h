#pragma once
#include <stdint.h>
#include "control_msg.h"

class EspNowLink {
public:
  bool beginSta();
  bool addPeer(const uint8_t peerMac[6]);
  bool sendToPeer(const uint8_t peerMac[6], const ControlMsg &msg);

private:
  static void onSent(const uint8_t *mac, esp_now_send_status_t status);
};
