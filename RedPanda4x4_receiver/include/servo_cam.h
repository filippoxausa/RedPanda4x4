#pragma once
#include <Arduino.h>

/// Initializes the camera servo.
void servoCamInit();

/// Sets the target angle (smoothing happens in servoCamUpdate).
void servoCamSetTarget(int angleDeg);

/// Call every loop: advances 1 degree toward the target every 35 ms.
void servoCamUpdate();

/// Current servo angle.
int servoCamCurrent();
