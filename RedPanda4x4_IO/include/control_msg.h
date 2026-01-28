#pragma once
#include <stdint.h>

typedef struct __attribute__((packed)) {
  int16_t speed; // -255..+255
} ControlMsg;
