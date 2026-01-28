#pragma once
#include <Arduino.h>
#include "control_msg.h"

class EspNowReceiver {
public:
  bool begin();
  void getLatest(ControlMsg &out, uint32_t &ageMs) const;

private:
  // Callback “vecchia” compatibile con molte versioni Arduino-ESP32
  static void onRecv(const uint8_t *mac, const uint8_t *data, int len);

  static volatile int16_t  s_ax, s_ay, s_az;
  static volatile uint32_t s_lastRxMs;
};
