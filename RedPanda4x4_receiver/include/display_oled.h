#pragma once
#include <Arduino.h>

/// Initializes the SSD1306 display. Returns false if not found.
bool displayInit();

/// Draws a happy face (autonomous driving, no obstacle).
void drawHappy();

/// Draws a sad face (obstacle detected).
void drawSad();

/// Draws the project name "RedPanda 4x4".
void drawProjectName();
