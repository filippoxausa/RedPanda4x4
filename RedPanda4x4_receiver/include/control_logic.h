#pragma once
#include <stdint.h>
#include "control_msg.h"

struct MotorCmd { int16_t left; int16_t right; };

/// Converte ControlMsg (steer/throttle) in comandi left/right
/// con deadzone, inversione assi, pivot e mixing arcade.
MotorCmd accelToMotorsManual(const ControlMsg &a);
