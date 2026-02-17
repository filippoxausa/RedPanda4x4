#pragma once
#include <stdint.h>

typedef struct __attribute__((packed)) {
  bool autoMode;
  int16_t ax, ay, az;
} ControlMsg;
