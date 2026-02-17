#pragma once
#include <Arduino.h>

/// Resets the state machine (back to A_FWD, servo centered).
void autoReset();

/// Executes one tick of autonomous driving. backObstacle = rear IR sensor.
void runAuto(bool backObstacle);

/// True if the car sees an obstacle (for the display).
bool autoIsObstacle();

/// True if the car is reversing (for the buzzer).
bool autoIsReversing();
