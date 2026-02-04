#pragma once
#include <Arduino.h>

void motorsInit();
void motorsSet(int16_t left, int16_t right); // -255..+255
void motorsStop();
