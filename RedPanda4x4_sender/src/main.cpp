#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>

#include "control_msg.h"
#include "joystick.h"
#include "mpu6500_reader.h"
#include "espnow_sender.h"

// =======================================================
//                    UART -> CYD (monitor)
// =======================================================
static const int UART_TX = 17;
static const int UART_RX = -1;
HardwareSerial Link(2);

// =======================================================
//                       ESP-NOW
// =======================================================
static const uint8_t RX_MAC[6] = { 0xE0,0x8C,0xFE,0x2E,0x96,0x7C };
static const uint8_t ESPNOW_CH = 1;

// =======================================================
//                     Peripherals
// =======================================================
static DriveJoystick driveJoy;
static CamJoystick   camJoy;
static Mpu6500Reader mpu;
static EspNowSender  nowTx;

// =======================================================
//                   Mode (3 states)
// =======================================================
enum Mode : uint8_t { MODE_JOYSTICK = 1, MODE_TILT = 2, MODE_AUTO = 3 };
static Mode g_mode = MODE_JOYSTICK;

// debounce SW1
static bool     g_btnPrev = false;
static uint32_t g_btnT0   = 0;
static const uint32_t BTN_DEBOUNCE_MS = 120;

// =======================================================
//                  Display helpers
// =======================================================
static const char* dirTextFromSteer(int steer) {
  if (steer > 2400)  return "RIGHT";
  if (steer < -2400) return "LEFT";
  return "STRAIGHT";
}

static const char* modeText(uint8_t m) {
  if (m == MODE_TILT) return "TILT";
  if (m == MODE_AUTO) return "AUTO";
  return "JOY";
}

// =======================================================
//                        setup()
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Joysticks (pin VRX, VRY, SW, deadzone)
  driveJoy.begin(34, 35, 33, 1300);
  driveJoy.calibrateCenter();

  camJoy.begin(32, 13, 1300);

  // UART -> CYD monitor
  Link.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println("[SENDER] UART: D,mode,btn,pitch,roll,dir");

  // MPU6500
  if (!mpu.begin(0x68, 21, 22, 400000)) {
    Serial.println("[MPU] Init FAILED");
  } else {
    Serial.println("[MPU] Init OK");
    mpu.calibrate(2000);
  }

  // WiFi + ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.print("[ESP-NOW] TX STA MAC: ");
  Serial.println(WiFi.macAddress());

  if (!nowTx.begin(RX_MAC, ESPNOW_CH)) {
    Serial.println("[ESP-NOW] init/peer failed");
    while (true) delay(1000);
  }

  Serial.println("[SENDER] Ready: SW1 cycles JOY->TILT->AUTO, SW2 centers cam");
}

// =======================================================
//                        loop()
// =======================================================
void loop() {
  static uint32_t lastMs = millis();
  uint32_t now = millis();
  float dt = (now - lastMs) / 1000.0f;
  if (dt <= 0.0f) dt = 0.01f;
  if (dt > 0.1f)  dt = 0.1f;
  lastMs = now;

  // ---- Read peripherals ----
  driveJoy.update();
  camJoy.update();
  bool imuOk = mpu.updateAngles(dt);

  bool btnPressed = driveJoy.button();

  // ---- Mode toggle on press (debounce) ----
  if (btnPressed != g_btnPrev) { g_btnPrev = btnPressed; g_btnT0 = now; }
  if ((now - g_btnT0) > BTN_DEBOUNCE_MS) {
    static bool lastStable = false;
    if (btnPressed != lastStable) {
      lastStable = btnPressed;
      if (lastStable) {
        if (g_mode == MODE_JOYSTICK)     g_mode = MODE_TILT;
        else if (g_mode == MODE_TILT)    g_mode = MODE_AUTO;
        else                             g_mode = MODE_JOYSTICK;
        Serial.print("[MODE] -> ");
        Serial.println(modeText(g_mode));
      }
    }
  }

  float pitchNow = mpu.pitchDeg();
  float rollNow  = mpu.rollDeg();

  // ---- Choose outputs based on mode ----
  bool autoMode = (g_mode == MODE_AUTO);
  int steer = 0, outAx = 0, outAy = 0;

  if (g_mode == MODE_JOYSTICK) {
    outAx = driveJoy.x();
    outAy = driveJoy.y();

    // Pivot: strong steer + low throttle -> force throttle to 0
    const int PIVOT_STEER = 4000;
    const int PIVOT_THR   = 3200;
    if (abs(outAx) > PIVOT_STEER && abs(outAy) < PIVOT_THR) {
      outAy = 0;
    }
    steer = outAx;

  } else if (g_mode == MODE_TILT) {
    const float PITCH_MAX_DEG = 25.0f;
    const float ROLL_MAX_DEG  = 25.0f;

    int pitchCmd = (int)lroundf((pitchNow / PITCH_MAX_DEG) * (float)OUTPUT_MAX);
    int rollCmd  = (int)lroundf((rollNow  / ROLL_MAX_DEG)  * (float)OUTPUT_MAX);

    pitchCmd = clampInt(pitchCmd, -OUTPUT_MAX, OUTPUT_MAX);
    rollCmd  = clampInt(rollCmd,  -OUTPUT_MAX, OUTPUT_MAX);
    pitchCmd = applyDeadzone(pitchCmd, 1000);
    rollCmd  = applyDeadzone(rollCmd,  1000);

    outAx = rollCmd;
    outAy = pitchCmd;
    steer = rollCmd;

  } else {
    // MODE_AUTO: nessun comando manuale
    outAx = 0; outAy = 0; steer = 0;
  }

  int outAz = camJoy.x();
  const char* dirTxt = dirTextFromSteer(steer);

  // UART verso CYD monitor
  Link.printf("D,%d,%d,%.1f,%.1f,%s\n",
              (int)g_mode, (btnPressed ? 1 : 0), pitchNow, rollNow, dirTxt);

  // ---- Invio 20 Hz via ESP-NOW ----
  static uint32_t lastSendMs = 0;
  if (now - lastSendMs >= 50) {
    lastSendMs = now;

    ControlMsg msg;
    msg.autoMode = autoMode;
    msg.ax = (int16_t)clampInt(outAx, -OUTPUT_MAX, OUTPUT_MAX);
    msg.ay = (int16_t)clampInt(outAy, -OUTPUT_MAX, OUTPUT_MAX);
    msg.az = (int16_t)clampInt(outAz, -OUTPUT_MAX, OUTPUT_MAX);

    nowTx.send(RX_MAC, msg);

    Serial.printf("[TX] mode=%s auto=%d | x=%d y=%d z=%d | pitch=%.1f roll=%.1f dir=%s imu=%s\n",
                  modeText(g_mode), (int)msg.autoMode,
                  (int)msg.ax, (int)msg.ay, (int)msg.az,
                  pitchNow, rollNow, dirTxt, imuOk ? "OK" : "FAIL");
  }

  delay(5);
}
