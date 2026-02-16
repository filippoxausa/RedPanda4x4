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
    -   Joystick 1: drive (X/Y) + push button (mode switch)
    -   Joystick 2: camera pan (X) + push button (center camera)
-   IMU **MPU6500** (I2C)
-   **CYD / TFT display board** (monitor) connected to controller via UART

**Vehicle (RX)**

-   ESP32 board (vehicle)
-   **TB6612FNG** motor driver + 4 DC motors (4WD chassis)
-   Obstacle sensors:
    -   IR sensor(s)
    -   Ultrasonic sensor + servo (scan)
-   Camera pan servo
-   OLED display + buzzer (vehicle feedback)

**Camera streaming**

-   ESP32-CAM (or ESP32 with camera support)
-   Camera module + LED flash

**Host PC (for vision demo)**

-   A laptop/PC to run the Python script (MJPEG decode + YOLO ONNX)


### Software requirements

**Firmware**

-   **VS Code + PlatformIO** (recommended)
    -   or Arduino IDE with ESP32 core installed
-   ESP32 board support package (Arduino core / PlatformIO platform)
-   Libraries used in the sketches/projects (typical):
    -   `esp_now`, `WiFi`, `Wire`
    -   `TFT_eSPI` (monitor display)
    -   `esp32-camera` components (camera server)

**Python (vision demo)**

-   Python 3.9+
-   Packages:
    -   `opencv-python`
    -   `numpy`
    -   `requests`
-   Model files in the same folder as the script:
    -   `yolov8n.onnx`
    -   `coco.names`

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
│   │   ├── app_httpd.cpp
│   │   ├── camera_pins.h
│   │   ├── export_onnx.py
│   │   ├── script.py
│   ├── macchina
│   │   └── macchina.ino
│   ├── receiverMonitor
│   │   └── receiverMonitor.ino    
│   └── senderJoystick
│       ├── app_httpd.cpp
│       ├── camera_pins.h
│       ├── ci.json
│       ├── partitions.csv
│       └── senderJoystick.ino
├── LICENSE
└── README.md
```

How to Build, Flash, and Run
-------------------------------------

### A) PlatformIO (recommended)

#### 1) Flash the controller (TX)

1.  Open `RedPanda4x4_sender/` in VS Code (PlatformIO).
2.  Select the correct **ESP32 board** and **serial port**.
3.  Build and upload:
    -   **PlatformIO GUI**: *Build* → *Upload*
    -   or CLI:
      ```
        pio run -t upload
      ```
4.  In the controller code, ensure these match your setup:
    -   Receiver MAC address (`RX_MAC`)
    -   ESPNOW channel (`ESPNOW_CH`)
    -   GPIO pins for joysticks/buttons/IMU/UART
  
#### 2) Flash the vehicle (RX)

1.  Open `RedPanda4x4_receiver/` in PlatformIO.
2.  Select the correct board/port.
3.  Upload:
    `pio run -t upload`

#### 3) Flash the monitor (optional)

-   Flash `src/receiverMonitor/receiverMonitor.ino` on the display board (CYD / TFT ESP32).

#### 4) Flash the camera (optional)

-   Build/flash the camera project that includes `app_httpd.cpp` (and WiFi credentials in the camera main file).
-   After boot, open in a browser:
    -   `http://<CAMERA_IP>/`
    -   Stream: `http://<CAMERA_IP>/stream`

### B) Arduino IDE (alternative)

1.  Open the sketch you want:
    -   Controller: `src/senderJoystick/senderJoystick.ino`
    -   Vehicle: `src/macchina/macchina.ino`
    -   Monitor: `src/receiverMonitor/receiverMonitor.ino`
2.  Select **Board = ESP32** variant and the correct COM port.
3.  Install required libraries (e.g., `TFT_eSPI`).
4.  Upload.

User Guide
----------

### Controller (TX) controls

-   **Mode button (Joystick 1 press)** cycles:
    1.  **JOYSTICK mode**: drive with joystick X/Y
    2.  **TILT mode**: drive by tilting the controller (IMU pitch/roll)
    3.  **AUTO mode**: vehicle runs autonomous FSM (controller sends auto flag)

-   **Drive joystick (Joystick 1)**
    -   X axis → steering
    -   Y axis → throttle forward/backward
    -   Deadzone + rescaling are applied for stability

-   **Camera joystick (Joystick 2)**

    -   X axis → camera pan command
    -   **Joystick 2 press** → instantly center camera pan

-   **Calibration**
    -   At startup, joystick center is calibrated automatically (keep sticks centered).

### Vehicle (RX) behavior

-   In **manual modes**, the vehicle follows joystick/tilt commands received over ESP-NOW.
-   In **AUTO mode**, the vehicle runs an **obstacle avoidance FSM**:
    -   drives forward until obstacle
    -   stops, scans, chooses a free direction
    -   turns or backs up if blocked

### Camera + Vision demo (optional)

1.  Make sure the camera stream is reachable:
    -   `http://<CAMERA_IP>/stream`
2.  On the PC, run:
    `python script.py`
3.  The script:
    -   decodes MJPEG frames
    -   (optionally) rotates frames
    -   runs YOLO ONNX every N frames for speed
    -   draws bounding boxes on the live video

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

-   **PowerPoint presentation**: *(add link here)*

-   **YouTube video demo**: *(add link here)*

Team Members & Contributions
----------------------------

-   **Member 1**:

-   **Member 2**:

-   **Member 3**:
