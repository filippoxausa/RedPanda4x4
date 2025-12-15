# RedPanda4x4
Embedded Systems Project – 4WD Telemetry & Control Platform

RedPanda4x4 is a 4WD vehicle platform controlled by a wireless “steering wheel” controller, with real-time bidirectional telemetry between the handheld unit and the car.

## Project status (updated)
- The project now uses **only two ESP32 boards** (MSP432 is no longer used).
- One ESP32 is inside the handheld steering/controller unit.
- One ESP32 is mounted on the car (motor + sensors + telemetry).

## System architecture

### Handheld controller (ESP32)
Responsibilities:
- Read motion input from an accelerometer/gyro (used to steer/drive the car remotely).
- Read a joystick (also used to drive/override motion control).
- Drive a small display (show live telemetry and status).
- Send commands to the car and receive telemetry back.

Typical commands sent to the car:
- Throttle / target speed
- Steering (from IMU tilt/rotation and/or joystick)
- Brake / emergency stop
- Mode selection (manual / assisted / line-following future)

### Car unit (ESP32)
Responsibilities:
- Receive commands from the handheld controller and apply them to the drivetrain.
- Control the motors through a TB6612FNG driver (PWM speed control + direction + braking).
- Read onboard sensors (initially ultrasonic + IR sensors) to help prevent collisions and/or constrain movement.
- Send telemetry back to the handheld controller (speed estimate, obstacle distance, sensor states, battery/voltage if available, link status).

## Communication (TBD)
Two supported directions are being evaluated:
- **ESP-NOW** for direct peer-to-peer communication between ESP32 devices without needing a router/access point.
- **Wi‑Fi** using an Access Point (AP) approach (e.g., UDP-based packets) for easier debugging/expansion (TBD).

Both options use a simple packet protocol (header + message type + payload + checksum/CRC), documented in `docs/protocol.md`.

## Control logic and modes
Planned high-level behavior includes a state machine, for example:
- `STOPPED`: motors disabled / safe state.
- `MANUAL_MOVE`: car moves purely from user commands (no sensor constraints).
- `ASSISTED_MOVE`: car moves from user commands but sensors can limit speed/force braking to avoid collisions.

Possible future modes:
- `LINE_FOLLOW`: follow a colored line on the ground using a dedicated line sensor (planned).

## Possible additions
- Telemetry logging via a small web server on the car ESP32 (or the controller ESP32) to record runs and sensor data.
- Line-following sensor integration and autonomous mode.
- Additional telemetry (battery monitor, car-side IMU, light sensor, etc.).
