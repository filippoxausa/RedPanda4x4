RedPanda4x4
===============================================================================

RedPanda4x4 is a small 4WD rover controlled by an ESP32-based handheld controller using **ESP-NOW**.\
The controller supports **manual driving** (joystick), **tilt driving** (IMU), and an **autonomous mode** (FSM on the vehicle).\
A separate camera module streams **MJPEG over HTTP**, and a Python script can run **YOLO (ONNX)** on the live stream.

Requirements
------------

### Hardware requirements

<p>
<img src="https://github.com/user-attachments/assets/df5ea54d-8d6e-41c7-a4a7-39ddf8c3f366" width="300px">
</p>

**Controller (TX)**
- ESP32 board (controller)
- analog joysticks
    - Joystick 1: drive (X/Y) + push button (mode switch)
    - Joystick 2: camera pan (X) + push button (center camera)
- IMU **MPU6500** (I2C)
- **CYD / TFT display board** (monitor) connected to controller via UART

**Vehicle (RX)**

- ESP32 board (vehicle)
- **TB6612FNG** motor driver + 4 DC motors (4WD chassis)
- Obstacle sensors:
  - IR sensor(s)
  - Ultrasonic sensor + servo (scan)
- Camera pan servo
- OLED display + buzzer (vehicle feedback)

**Camera streaming**

- ESP32-CAM (or ESP32 with camera support)
- Camera module + LED flash

**Host PC (for vision demo)**

- A laptop/PC to run the Python script (MJPEG decode + YOLO ONNX)

---

### Wiring / Pinout

> Important: **All grounds are in common** (ESP32, TB6612FNG, sensors, servos, buzzer, OLED).  
> Power note (current build): a **4×AA battery pack** powers **motors + servos + HC-SR04**, while the ESP32 **3.3V rail** powers the **logic / low-power peripherals**.

#### Vehicle (RX) wiring (ESP32 + TB6612FNG + sensors)

**TB6612FNG motor driver (control pins to ESP32 + motor outputs)**
| TB6612FNG Pin | Connects to | Notes |
|---|---|---|
| AIN1 | ESP32 GPIO14 | Motor A direction (Left) |
| AIN2 | ESP32 GPIO27 | Motor A direction (Left) |
| PWMA | ESP32 GPIO26 | Motor A PWM (Left) |
| BIN1 | ESP32 GPIO16 | Motor B direction (Right) |
| BIN2 | ESP32 GPIO17 | Motor B direction (Right) |
| PWMB | ESP32 GPIO4 | Motor B PWM (Right) |
| STBY | ESP32 GPIO25 | Driver enable (HIGH = ON) |
| AO1 / AO2 | Left motor terminals | TB6612FNG output to left motor side (motor wiring order sets direction) |
| BO1 / BO2 | Right motor terminals | TB6612FNG output to right motor side (motor wiring order sets direction) |
| VM | + 4×AA battery pack | Motor power input |
| VCC | ESP32 3.3V | Logic power (recommended 3.3V for input compatibility) |
| GND | Common GND | Must be shared with ESP32 and battery pack |

**Ultrasonic sensor (HC-SR04)**
| HC-SR04 Pin | Connects to | Notes |
|---|---|---|
| TRIG | ESP32 GPIO18 | Digital output |
| ECHO | ESP32 GPIO19 | **ECHO is 5V** → we are not using a **voltage divider / level shifter** to 3.3V |
| VCC | + 4×AA battery pack | Sensor power |
| GND | Common GND | Shared ground |

**Rear IR obstacle sensor (digital)**
| IR Sensor Pin | Connects to | Notes |
|---|---|---|
| OUT | ESP32 GPIO23 | Read with `INPUT_PULLUP` (active LOW) |
| VCC | ESP32 3.3V | - |
| GND | Common GND | Shared ground |

**OLED I2C (SSD1306 128×64)**
| OLED Pin | Connects to | Notes |
|---|---|---|
| SDA | ESP32 GPIO21 | I2C SDA (`Wire.begin(21, 22)`) |
| SCL | ESP32 GPIO22 | I2C SCL |
| VCC | ESP32 3.3V | Power |
| GND | Common GND | Shared ground |
| I2C address | 0x3C | If not detected, try 0x3D |

**Passive buzzer**
| Buzzer Pin | Connects to | Notes |
|---|---|---|
| + | ESP32 GPIO5 | Driven with LEDC tone |
| − | Common GND | Shared ground |

**Servos**
| Servo | Signal Pin (ESP32) | Power | Notes |
|---|---:|---|---|
| Ultrasonic scan servo (MG 90g) | GPIO33 | ESP32 3.3V (as wired) | PWM signal from ESP32 |
| Camera pan servo | GPIO32 | ESP32 3.3V (as wired) | PWM signal from ESP32 |

**Power rails summary (RX)**
| Rail | Source | Powers |
|---|---|---|
| ~6V / ~4.8V | 4×AA battery pack | TB6612FNG **VM** (motors), HC-SR04 **VCC**, (optional) servos |
| 3.3V | ESP32 3.3V pin | TB6612FNG **VCC** (logic), OLED, IR sensor, buzzer/signal, (currently) servos |
| GND | Common | Must connect all modules together |

#### Controller (TX) wiring (ESP32 + joysticks + MPU6500 + monitor UART)

**UART to CYD/TFT monitor board**
| Signal | ESP32 Pin | Notes |
|---|---:|---|
| TX (ESP32 → Monitor RX) | GPIO17 | `HardwareSerial(2)`, `Link.begin(..., RX=-1, TX=17)` |
| RX (Monitor TX → ESP32) | Not used | `UART_RX = -1` |

**Joystick 1 (Drive)**
| Joystick 1 Pin | ESP32 Pin | Notes |
|---|---:|---|
| VRX (X axis) | GPIO34 (ADC1) | Analog read |
| VRY (Y axis) | GPIO35 (ADC1) | Analog read |
| SW (button) | GPIO33 | Digital input, **LOW = pressed**, `INPUT_PULLUP` |

**Joystick 2 (Camera pan + center)**
| Joystick 2 Pin | ESP32 Pin | Notes |
|---|---:|---|
| VRX (X axis) | GPIO32 (ADC1) | Analog read (OK with WiFi) |
| SW (button) | GPIO13 | Digital input, **LOW = pressed**, `INPUT_PULLUP` |

**MPU6500 (I2C)**
| MPU6500 Pin | ESP32 Pin | Notes |
|---|---:|---|
| SDA | GPIO21 | `Wire.begin(21, 22)` |
| SCL | GPIO22 | `Wire.setClock(400000)` |
| VCC | 3.3V | Sensor power |
| GND | GND | Shared ground |
| I2C addr | 0x68 | `MPU_ADDR = 0x68` |

---

### Software requirements

**Firmware**

- **VS Code + PlatformIO** 
- ESP32 board support package (PlatformIO platform)
- Libraries used in the sketches/projects (typical):
  - `esp_now`, `WiFi`, `Wire`
  - `TFT_eSPI` (monitor display)
  - `esp32-camera` components (camera server)

**Python (vision demo)**

- Python 3.9+
- Packages:
  - `opencv-python`
  - `numpy`
  - `requests`
- Model files in the same folder as the script:
  - `yolov8n.onnx`
  - `coco.names`

Install Python deps:
```
pip install opencv-python numpy requests
```

Project Layout
--------------
```
RedPanda4x4
├── RedPanda4x4_receiver              # RedPanda4x4_receiver → vehicle firmware (RX)
│   ├── include
│   │   ├── auto_drive.h
│   │   ├── buzzer.h
│   │   ├── control_logic.h
│   │   ├── control_msg.h
│   │   ├── display_oled.h
│   │   ├── espnow_receiver.h
│   │   ├── helpers.h
│   │   ├── ir_sensor.h
│   │   ├── motor_tb6612.h
│   │   ├── servo_cam.h
│   │   └── servo_ultrasonic.h
│   ├── src
│   │   ├── auto_drive.cpp            # autonomous FSM (Forward / Stop / Scan / Back / Turn)
│   │   ├── buzzer.cpp                # buzzer feedback
│   │   ├── control_logic.cpp         # mixes manual/autonomous commands into actuators
│   │   ├── display_oled.cpp          # vehicle OLED feedback
│   │   ├── espnow_receiver.cpp       # ESP-NOW receive callback + latest message storage
│   │   ├── ir_sensor.cpp             # IR obstacle detection
│   │   ├── main.cpp
│   │   ├── motor_tb6612.cpp          # TB6612 motor control
│   │   ├── servo_cam.cpp             # camera pan servo control
│   │   └── servo_ultrasonic.cpp      # ultrasonic scan servo control
├── RedPanda4x4_sender                # RedPanda4x4_sender → controller firmware (TX)
│   ├── include
│   │   ├── control_msg.h
│   │   ├── espnow_sender.h
│   │   ├── joystick.h
│   │   └── mpu6500_reader.h
│   ├── src
│   │   ├── espnow_sender.cpp         # ESP-NOW setup + send + send-callback
│   │   ├── joystick.cpp              # Joystick reading, deadzones, mapping
│   │   ├── main.cpp
│   │   └── mpu6500_reader.cpp        # IMU init + complementary filter
├── src                               # Arduino-style copies / additional sketches (camera, monitor, etc.)
│   ├── cam
│   │   ├── camera_pins.h
│   │   ├── script.py
│   ├── receiverMonitor
│       └── receiverMonitor.ino
├── LICENSE
└── README.md
```

How to Build, Flash, and Run
-------------------------------------

### PlatformIO (recommended)

#### 1) Flash the controller (TX)

1. Open `RedPanda4x4_sender/` in VS Code (PlatformIO).
2. Select the correct **ESP32 board** and **serial port**.
3. Build and upload:
   - **PlatformIO GUI**: *Build* → *Upload*
   - or CLI:
     ```
     pio run -t upload
     ```
4. In the controller code, ensure these match your setup:
   - Receiver MAC address (`RX_MAC`)
   - ESPNOW channel (`ESPNOW_CH`)
   - GPIO pins for joysticks/buttons/IMU/UART

#### 2) Flash the vehicle (RX)

1. Open `RedPanda4x4_receiver/` in PlatformIO.
2. Select the correct board/port.
3. Upload:
   `pio run -t upload`

#### 3) Flash the monitor (optional)

- Flash `src/receiverMonitor/receiverMonitor.ino` on the display board (CYD / TFT ESP32).

#### 4) Flash the camera (optional)

- Build/flash the camera project that includes `app_httpd.cpp` (and WiFi credentials in the camera main file).
- After boot, open in a browser:
  - `http://<CAMERA_IP>/`
  - Stream: `http://<CAMERA_IP>/stream`

User Guide
----------

### Controller (TX) controls

- **Mode button (Joystick 1 press)** cycles:
  1. **JOYSTICK mode**: drive with joystick X/Y
  2. **TILT mode**: drive by tilting the controller (IMU pitch/roll)
  3. **AUTO mode**: vehicle runs autonomous FSM (controller sends auto flag)

- **Drive joystick (Joystick 1)**
  - X axis → steering
  - Y axis → throttle forward/backward
  - Deadzone + rescaling are applied for stability

- **Camera joystick (Joystick 2)**
  - X axis → camera pan command
  - **Joystick 2 press** → instantly center camera pan

- **Calibration**
  - At startup, joystick center is calibrated automatically (keep sticks centered).

### Vehicle (RX) behavior

- In **manual modes**, the vehicle follows joystick/tilt commands received over ESP-NOW.
- In **AUTO mode**, the vehicle runs an **obstacle avoidance FSM**:
  - drives forward until obstacle
  - stops, scans, chooses a free direction
  - turns or backs up if blocked

### Camera + Vision demo (optional)

1. Make sure the camera stream is reachable:
   - `http://<CAMERA_IP>/stream`
2. On the PC, run:
   `python script.py`
3. The script:
   - decodes MJPEG frames
   - (optionally) rotates frames
   - runs YOLO ONNX every N frames for speed
   - draws bounding boxes on the live video

## Autonomous Drive Pipeline (FSM)

The autonomous driving logic is implemented as a **Finite State Machine (FSM)**.
At each iteration of the main loop, the robot reads the environment (mainly the **ultrasonic distance sensor**) and updates its behavior by switching between a small set of states.

### FSM States
<img width="754" height="331" alt="image-removebg-preview" src="https://github.com/user-attachments/assets/2967058f-1a31-42fa-a154-c41fee2ad96c" />

The FSM is defined as:
`enum AutoState : uint8_t { A_FWD=0, A_STOP, A_SCAN, A_BACK, A_TURN };`

**Meaning of each state:**<br>
A_FWD (Forward): Drive forward with the ultrasonic sensor aligned to the front. Continuously check the front distance.<br>
A_STOP (Stop): Immediately stop the motors when an obstacle is detected. This is a "safety" state before deciding the next maneuver.<br>
A_SCAN (Scan left/right): Use the servo-mounted ultrasonic sensor to scan the environment (typically center → left → right) and measure free space.<br>
A_TURN (Turn): Turn toward the direction that was detected as free during the scan (left or right).<br>
A_BACK (Reverse): If no safe direction is found, reverse for a short time to create space and retry.<br>

**Transitions (High-level logic):**<br>
A_FWD → A_STOP when a front obstacle is closer than a threshold.<br>
A_STOP → A_SCAN after a short stabilization delay.<br>
A_SCAN → A_TURN if at least one direction (left/right) is free.<br>
A_SCAN → A_BACK if no direction is free.<br>
A_BACK → A_SCAN after reversing for a fixed duration (retry the scan).<br>
A_TURN → A_FWD after completing the turn (resume forward motion).

Links
-----

- **PowerPoint presentation**: *(https://docs.google.com/presentation/d/1fTfeVSfemBiTMKqylM_2xex5caF504tukKaZ9YwD_rQ/edit?usp=sharing)*
- **YouTube video demo**: *(www.youtube.com/watch?v=iV_uJD73mQ0 due to privacy problems our channel is currently unavailable, we'll fix in the next days if it's necessary)*

**Team members**: *Boarini Andrea*, *Calliari Michele*, *Carbonari Ismaele*, *Xausa Filippo*
