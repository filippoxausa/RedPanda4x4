# RedPanda4x4 🐼🚗  
Embedded Systems Project – 4WD Telemetry & Control Platform

---

## 📌 Overview

**RedPanda4x4** is a 4WD platform designed as an Embedded Systems project:  
a **four-wheel-drive vehicle** controlled by a **wireless steering wheel** with sensors and real-time telemetry.

Main idea:

- In your hands you have a **steering controller** with:
  - an **MSP432** (or ESP32, depending on the chosen architecture),
  - a **gyroscope** to measure the steering angle,
  - a **display** to show speed and other real-time data.
- On the car you have:
  - another **board (MSP432 or ESP32)** that controls the motors via **TB6612FNG**,
  - various **sensors** (ultrasonic sensor, temperature sensor, line follower, solar panel, etc.),
  - an **ESP32** for the wireless link with the steering wheel.

The goal is to implement:
- full **4WD control** (forward/backward, turning, braking),
- **bidirectional telemetry** (commands from steering wheel → car, sensor data from car → steering wheel),
- a **modular system** that can be extended with new sensors/features.

---

## 🧱 Possible Architectures

We are considering two main architectures for splitting roles between **MSP432** and **ESP32**.

### 🔹 Option A – MSP432 in the steering wheel, ESP32 on the car

**Steering wheel (in hand)**  
- **MSP432** as main MCU:
  - reads the **gyroscope** (steering angle),
  - handles user inputs (throttle, brake, mode selection),
  - updates the **display** (speed, battery status, sensor data, etc.),
  - communicates over **UART** with a locally connected **ESP32**.
- **ESP32 (steering side)**:
  - receives data from the MSP432 via UART,
  - sends commands to the car through a **wireless** link (e.g. Wi-Fi or ESP-NOW),
  - receives telemetry from the car and forwards it to the MSP432.

**Car (4x4)**  
- **ESP32 (car side)**:
  - receives control commands from the steering wheel,
  - sends sensor data and system status back to the steering wheel,
  - acts as a “bridge” between the wireless link and the motor/sensor controller (if a separate MSP is used).
- **MSP432 (car side, optional)**:
  - performs **motor control** using the **TB6612FNG** driver,
  - reads all **on-board sensors** (ultrasonic, line follower, temperature, solar panel, etc.),
  - computes derived quantities (e.g. speed, estimated distance).
- **Motor driver**:
  - **TB6612FNG** connected to the 4WD chassis motors (non-steering chassis, turning is done by differential wheel control).
- **On-board sensors (modular list)**:
  - **Ultrasonic sensor** (for obstacle detection and distance measurement),
  - **Temperature sensor** (ambient or motor/battery temperature),
  - **Solar panel** with voltage/current sensing (power harvesting monitoring),
  - **Line follower** (IR sensor array to follow a line on the ground),
  - Additional sensors (optional): IMU on the car, light sensor, battery monitor, etc.

---

### 🔹 Option B – ESP32 in the steering wheel, MSP432 on the car

Alternatively, we can swap roles:

**Steering wheel (in hand)**  
- **ESP32**:
  - directly reads the **gyroscope** and user inputs,
  - manages the **display** and user interface logic,
  - communicates **wirelessly** directly with the car (no UART needed on the steering side).
  
**Car (4x4)**  
- **MSP432**:
  - receives commands from the ESP32 via UART (local link on the car),
  - handles the **TB6612FNG** motor driver,
  - reads all **on-board sensors**,
  - sends telemetry data back to the ESP32 (which then forwards it to the steering wheel).

This option uses the ESP32 mainly as *communication and UI unit*, while the MSP432 focuses on “bare-metal” control of actuators and sensors.

---

## 🧩 Main Features

### 🎮 Steering Wheel / Controller

- **Steering angle** acquisition via gyroscope.
- Control of:
  - speed (throttle),
  - direction (left/right),
  - brake / emergency stop,
  - modes (e.g. Eco, Sport, Auto line-following).
- **Display**:
  - estimated vehicle speed,
  - distance to obstacles,
  - battery status,
  - temperature (e.g. ambient),
  - connection status (link OK / lost).
- **Communication**:
  - UART between MSP432 ↔ ESP32 (if we use Option A),
  - wireless link between steering wheel ↔ car (Wi-Fi/ESP-NOW).

### 🚗 4WD Car

- **Motor control** via TB6612FNG:
  - independent control of left/right sides (or each wheel),
  - PWM regulation for speed control,
  - implementation of fast braking / emergency stop.
- **Main sensors**:
  - Ultrasonic sensor for obstacle detection (e.g. front distance),
  - Line follower for following a predefined path on the ground,
  - Temperature sensor (environment or components),
  - Solar panel monitoring (generated voltage/current),
  - Battery monitor (remaining voltage).
- Possible modes:
  - **Manual**: direct command from the steering wheel,
  - **Assisted**: system automatically limits speed based on obstacles,
  - **Line-Following**: vehicle follows the line autonomously, steering wheel may control only the speed.

---

## 🔌 Communication

- **UART**:
  - between MSP432 and ESP32 (when present on the same unit),
  - simple protocol (e.g. packets with header, message type, payload, checksum).
- **Wireless (ESP32 ↔ ESP32 or ESP32 ↔ MSP)**:
  - Evaluation of:
    - **ESP-NOW** for low-latency peer-to-peer communication,
    - or **Wi-Fi** with a custom protocol (e.g. UDP).
- **Telemetry**:
  - From steering wheel → car:
    - steering angle, desired speed, operating mode.
  - From car → steering wheel:
    - current speed, obstacle distance, temperature, battery status, solar panel output, sensor status (line detected/not detected, etc.).

---

## 🧪 Project Goals

- Design and build a complete **embedded system** (hardware + firmware).
- Implement:
  - **real-time motor control**,
  - acquisition and handling of **heterogeneous sensors**,
  - **reliable communication** between remote units.
- Integrate everything into a **working prototype** with an intuitive user interface (steering wheel with display and feedback).

---

## 📁 Repository Structure (proposed)

```text
RedPanda4x4/
├─ docs/
│  ├─ schematics/
│  ├─ diagrams/
│  └─ protocol.md
├─ firmware/
│  ├─ steering_msp432/
│  ├─ steering_esp32/
│  ├─ car_msp432/
│  └─ car_esp32/
├─ hardware/
│  ├─ pcb/
│  └─ wiring/
└─ README.md

