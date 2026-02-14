#pragma once
#include <Arduino.h>

/// Inizializza il display SSD1306. Ritorna false se non trovato.
bool displayInit();

/// Disegna faccina felice (guida autonoma, nessun ostacolo).
void drawHappy();

/// Disegna faccina triste (ostacolo rilevato).
void drawSad();

/// Disegna il nome del progetto "RedPanda 4x4".
void drawProjectName();
