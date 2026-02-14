#pragma once
#include <Arduino.h>

/// Inizializza il servo del sensore ultrasuoni.
void servoUsInit();

/// Posiziona il servo (limitato a min..max).
void servoUsWrite(int angleDeg);

/// Inizializza i pin del HC-SR04.
void hcsr04Init();

/// Legge la distanza in cm (999 se fuori range / timeout).
uint16_t hcsr04ReadCm();

/// Satura distanze fuori [1..400] a 999.
uint16_t clampDist(uint16_t d);

// Angle getters
int servoUsCenterAngle();
int servoUsMinAngle();
int servoUsMaxAngle();
