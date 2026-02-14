#pragma once
#include <Arduino.h>

/// Inizializza pin motori + LEDC + standby HIGH.
void motorsInit();

/// Comanda direttamente i motori senza rampa (-255..+255).
void motorsRaw(int16_t left, int16_t right);

/// Comanda i motori con rampa anti-slip + limitazione differenziale.
void motorsSetSmooth(int16_t targetL, int16_t targetR);

/// Equivale a motorsSetSmooth(0, 0).
void motorsStop();
