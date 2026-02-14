#pragma once
#include <Arduino.h>

/// Resetta la macchina a stati (torna in A_FWD, servo al centro).
void autoReset();

/// Esegue un tick della guida autonoma. backObstacle = sensore IR posteriore.
void runAuto(bool backObstacle);

/// True se l'auto vede un ostacolo (per il display).
bool autoIsObstacle();

/// True se l'auto sta andando in retromarcia (per il buzzer).
bool autoIsReversing();
