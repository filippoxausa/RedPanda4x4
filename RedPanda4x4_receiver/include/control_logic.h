#pragma once
#include <stdint.h>
#include "control_msg.h"

struct MotorCmd { int16_t left; int16_t right; };

/// Converts ControlMsg (steer/throttle) into left/right commands
/// with deadzone, axis inversion, pivot and arcade mixing.
MotorCmd accelToMotorsManual(const ControlMsg &a);
