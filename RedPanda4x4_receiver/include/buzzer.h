#pragma once
#include <Arduino.h>

/// Inizializza il buzzer (LEDC).
void buzzerInit();

/// Chiama ogni loop: se wantBeep=true emette beep intermittente (retromarcia).
void buzzerUpdate(bool wantBeep);
