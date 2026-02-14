#pragma once
#include <Arduino.h>

/// Inizializza il sensore IR posteriore.
void irInit();

/// Ritorna true se il sensore rileva un ostacolo dietro.
bool irBackObstacle();
