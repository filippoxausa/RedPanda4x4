#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <esp_arduino_version.h>
#include "control_msg.h"

class EspNowReceiver {
public:
  bool begin();
  void getLatest(ControlMsg &out, uint32_t &ageMs) const;

private:
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  static void onRecv(const esp_now_recv_info_t *info,
                     const uint8_t *data, int len);
#else
  static void onRecv(const uint8_t *mac,
                     const uint8_t *data, int len);
#endif

  static volatile bool     s_auto;
  static volatile int16_t  s_ax, s_ay, s_az;
  static volatile uint32_t s_lastRxMs;
};
