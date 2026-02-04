#pragma once
#include <stdint.h>
#include "control_msg.h"

struct MotorCmd { int16_t left; int16_t right; };

// Converte ax/ay in comandi left/right (curve + spin)
MotorCmd accelToMotors(const ControlMsg &a);
