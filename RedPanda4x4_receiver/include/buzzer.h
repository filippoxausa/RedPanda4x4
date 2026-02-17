#pragma once
#include <Arduino.h>

/// Initializes the buzzer (LEDC).
void buzzerInit();

/// Call every loop: if wantBeep=true, emits intermittent beep (reverse).
void buzzerUpdate(bool wantBeep);
