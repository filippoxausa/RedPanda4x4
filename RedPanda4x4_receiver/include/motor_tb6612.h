#pragma once
#include <Arduino.h>

/// Initializes motor pins + LEDC + standby HIGH.
void motorsInit();

/// Drives motors directly without ramp (-255..+255).
void motorsRaw(int16_t left, int16_t right);

/// Drives motors with anti-slip ramp + differential limiting.
void motorsSetSmooth(int16_t targetL, int16_t targetR);

/// Equivalent to motorsSetSmooth(0, 0).
void motorsStop();
