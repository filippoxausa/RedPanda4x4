#include <Arduino.h>
#include <WiFi.h>
#include "control_msg.h"
#include "mpu6500_reader.h"
#include "espnow_sender.h"
#include <math.h>

// ================= UART -> CYD =================
static const int UART_TX = 17;
static const int UART_RX = -1;
HardwareSerial Link(2);

// ================= Peer MAC (receiver) =========
static const uint8_t RX_MAC[6] = { 0xE0,0x8C,0xFE,0x2E,0x96,0x7C };

// ================= Joystick pins ===============
static const int PIN_VRX = 34;   // joy1 X
static const int PIN_VRY = 35;   // joy1 Y
static const int PIN_SW  = 33;   // joy1 button (mode toggle)

static const int PIN2_VRX = 32;  // joy2 X (cam pan)
static const int PIN2_SW  = 13;  // joy2 button (center cam) optional

// ================= MPU ==========================
static const uint8_t MPU_ADDR = 0x68;
static const float GYRO_SENS_250 = 131.0f; // raw/131 = dps

static Mpu6500Reader mpu;
static EspNowSender nowTx;

// ================= Mode =========================
enum Mode : uint8_t { MODE_JOYSTICK = 1, MODE_GYRO = 2 };
static Mode g_mode = MODE_JOYSTICK;

static bool     g_btnPrev = false;
static uint32_t g_btnT0   = 0;
static const uint32_t BTN_DEBOUNCE_MS = 120;

// ================= Calibration ==================
static int x1Center = 2048, y1Center = 2048, x2Center = 2048;
static float headingDeg = 0.0f;

// ---- helpers ----
static int clamp100(int v) {
  if (v < -100) return -100;
  if (v >  100) return  100;
  return v;
}

static int applyDeadzone100(int v, int dz) {
  if (v > -dz && v < dz) return 0;
  return v;
}

static int mapRelTo100(int raw, int center) {
  int d = raw - center;
  if (d >= 0) {
    int denom = (4095 - center);
    if (denom <= 0) return 0;
    return (int)lroundf((d * 100.0f) / denom);
  } else {
    int denom = center;
    if (denom <= 0) return 0;
    return (int)lroundf((d * 100.0f) / denom);
  }
}

static int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static const char* dirTextFromSteer(int steer) {
  if (steer > 15)  return "RIGHT";
  if (steer < -15) return "LEFT";
  return "STRAIGHT";
}

static void calibrateCenters() {
  const int N = 200;
  long sx1 = 0, sy1 = 0, sx2 = 0;

  // warmup ADC
  for (int i = 0; i < 20; i++) {
    (void)analogRead(PIN_VRX);
    (void)analogRead(PIN_VRY);
    (void)analogRead(PIN2_VRX);
    delay(2);
  }

  for (int i = 0; i < N; i++) {
    sx1 += analogRead(PIN_VRX);
    sy1 += analogRead(PIN_VRY);
    sx2 += analogRead(PIN2_VRX);
    delay(2);
  }

  x1Center = (int)(sx1 / N);
  y1Center = (int)(sy1 / N);
  x2Center = (int)(sx2 / N);

  Serial.printf("[CAL] joy1 center x=%d y=%d | joy2 center x=%d\n", x1Center, y1Center, x2Center);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_SW, INPUT_PULLUP);
  pinMode(PIN2_SW, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Serial.println("[CAL] Calibrating centers... DO NOT touch sticks");
  calibrateCenters();

  Link.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.print("[ESP] TX STA MAC: ");
  Serial.println(WiFi.macAddress());

  // MPU: usa clock più stabile (100k). Se vuoi, poi risali.
  mpu.begin(MPU_ADDR, 21, 22, 100000);

  if (!nowTx.begin(RX_MAC)) {
    Serial.println("[ESP-NOW] init/peer failed");
    while (true) delay(1000);
  }

  Serial.println("[SENDER] Ready: SW1 toggles mode, SW2 centers cam (optional)");
}

void loop() {
  static uint32_t lastMs = millis();
  uint32_t now = millis();
  float dt = (now - lastMs) / 1000.0f;
  if (dt <= 0) dt = 0.02f;
  lastMs = now;

  // ---- joystick 1 raw ----
  int xRaw = analogRead(PIN_VRX);
  int yRaw = analogRead(PIN_VRY);
  bool btnPressed = (digitalRead(PIN_SW) == LOW);

  // toggle mode debounce
  if (btnPressed != g_btnPrev) { g_btnPrev = btnPressed; g_btnT0 = now; }
  if ((now - g_btnT0) > BTN_DEBOUNCE_MS) {
    static bool lastStable = false;
    if (btnPressed != lastStable) {
      lastStable = btnPressed;
      if (lastStable) g_mode = (g_mode == MODE_JOYSTICK) ? MODE_GYRO : MODE_JOYSTICK;
    }
  }

  // ---- gyro Z ----
  float yawDps = 0.0f;
  int16_t gzRaw = 0;
  if (mpu.readGyroZRaw(gzRaw)) {
    yawDps = (float)gzRaw / GYRO_SENS_250;
    if (fabsf(yawDps) < 1.5f) yawDps = 0.0f;

    headingDeg += yawDps * dt;
    if (headingDeg > 180.0f) headingDeg -= 360.0f;
    if (headingDeg < -180.0f) headingDeg += 360.0f;
  } else {
    yawDps = 0.0f;
  }

  // ---- joystick 1 mapped (relative + deadzone) ----
  int xMapped = clamp100(mapRelTo100(xRaw, x1Center));
  int yMapped = clamp100(mapRelTo100(yRaw, y1Center));
  xMapped = applyDeadzone100(xMapped, 6);
  yMapped = applyDeadzone100(yMapped, 6);

  // ---- joystick 2 (cam pan) ----
  int camRawX = analogRead(PIN2_VRX);
  int camMappedX = clamp100(mapRelTo100(camRawX, x2Center));
  camMappedX = applyDeadzone100(camMappedX, 8);

  bool camCenterPressed = (digitalRead(PIN2_SW) == LOW);
  if (camCenterPressed) camMappedX = 0;

  // ---- choose ax/ay based on mode ----
  const int SCALE = 162;

  int steer = 0;
  int outAx = 0;
  int outAy = 0;

  if (g_mode == MODE_JOYSTICK) {
    steer = xMapped;
    outAx = xMapped;
    outAy = yMapped;
  } else {
    const float YAW_MAX_DPS = 120.0f;
    int yawMapped = (int)lroundf((yawDps / YAW_MAX_DPS) * 100.0f);
    yawMapped = clampInt(yawMapped, -100, 100);
    steer = yawMapped;
    outAx = yawMapped;
    outAy = yMapped;
  }

  int outAz = camMappedX; // cam pan

  // ---- UART ----
  const char* dirTxt = dirTextFromSteer(steer);
  Link.printf("D,%d,%d,%.1f,%.1f,%s\n",
              (int)g_mode, (btnPressed ? 1 : 0), yawDps, headingDeg, dirTxt);

  // ---- ESP-NOW 20 Hz ----
  static uint32_t lastSendMs = 0;
  if (now - lastSendMs >= 50) {
    lastSendMs = now;

    ControlMsg msg;
    msg.ax = (int16_t)(outAx * SCALE);
    msg.ay = (int16_t)(outAy * SCALE);
    msg.az = (int16_t)(outAz * SCALE);

    nowTx.send(RX_MAC, msg);

    Serial.printf("[TX] mode=%d | xM=%d yM=%d camM=%d | ax=%d ay=%d az=%d\n",
                  (int)g_mode, xMapped, yMapped, camMappedX,
                  (int)msg.ax, (int)msg.ay, (int)msg.az);
  }

  delay(5);
}
