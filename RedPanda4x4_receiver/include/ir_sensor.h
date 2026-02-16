#pragma once
#include <Arduino.h>

/// Initializes the rear IR sensor.
void irInit();

/// Returns true if the sensor detects a rear obstacle.
bool irBackObstacle();
