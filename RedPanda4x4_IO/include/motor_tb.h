#pragma once
#include <Arduino.h>

void motorsInit();
void motorEnable(bool en);

void driveA(int speed);   // -255..+255
void driveB(int speed);   // -255..+255
void brakeAll();
