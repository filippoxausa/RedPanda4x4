#pragma once
#include <Arduino.h>

/// Initializes the ultrasonic sensor servo.
void servoUsInit();

/// Positions the servo (clamped to min..max).
void servoUsWrite(int angleDeg);

/// Initializes the HC-SR04 pins.
void hcsr04Init();

/// Reads the distance in cm (999 if out of range / timeout).
uint16_t hcsr04ReadCm();

/// Clamps distances outside [1..400] to 999.
uint16_t clampDist(uint16_t d);

// Angle getters
int servoUsCenterAngle();
int servoUsMinAngle();
int servoUsMaxAngle();
