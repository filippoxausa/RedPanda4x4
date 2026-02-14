#pragma once
#include <Arduino.h>

/// Inizializza il servo della camera.
void servoCamInit();

/// Imposta l'angolo target (lo smoothing avviene in servoCamUpdate).
void servoCamSetTarget(int angleDeg);

/// Chiama ogni loop: avanza di 1 grado verso il target ogni 35 ms.
void servoCamUpdate();

/// Angolo attuale del servo.
int servoCamCurrent();
