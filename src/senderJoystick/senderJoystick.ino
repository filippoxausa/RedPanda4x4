#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

// =======================================================
//                    UART -> CYD (monitor)
// =======================================================
static const int UART_TX = 17;
static const int UART_RX = -1;
HardwareSerial Link(2);

// =======================================================
//                       JOYSTICK 1 (DRIVE)
// =======================================================
static const int PIN_VRX = 34;   // ADC1
static const int PIN_VRY = 35;   // ADC1
static const int PIN_SW  = 33;   // LOW=pressed

// =======================================================
//                       JOYSTICK 2 (CAM PAN)
// =======================================================
static const int PIN2_VRX = 32;  // ADC1 (OK with WiFi)
static const int PIN2_SW  = 13;  // LOW=pressed

// =======================================================
//                      MPU6500 (I2C)
// =======================================================
static const uint8_t MPU_ADDR = 0x68;

// Registers
static const uint8_t REG_WHO_AM_I     = 0x75;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_SMPLRT_DIV   = 0x19;
static const uint8_t REG_CONFIG       = 0x1A;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;

static const uint8_t REG_ACCEL_XOUT_H = 0x3B; // accel(6) temp(2) gyro(6)

// Sensibilità (config: accel ±2g, gyro ±250 dps)
static const float ACC_SENS_2G   = 16384.0f; // LSB/g
static const float GYRO_SENS_250 = 131.0f;   // LSB/(deg/s)

// Complementary filter
static const float RAD2DEG = 57.2957795f;
static const float ALPHA   = 0.98f;

// Angoli stimati
static float pitchDeg = 0.0f; // avanti/indietro
static float rollDeg  = 0.0f; // sinistra/destra

// Offsets
static float gxOff = 0, gyOff = 0, gzOff = 0;
static float axOff = 0, ayOff = 0, azOff = 0;
static float azSign = 1.0f;

// =======================================================
//                       ESP-NOW
// =======================================================
static const uint8_t RX_MAC[6] = { 0xE0,0x8C,0xFE,0x2E,0x96,0x7C };
static const uint8_t ESPNOW_CH = 1;

typedef struct __attribute__((packed)) {
  bool autoMode;
  int16_t ax, ay, az;
} ControlMsg;

static void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  (void)mac;
  Serial.print("[ESP-NOW] send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// =======================================================
//                   I2C helpers (MPU)
// =======================================================
static bool i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; i++) data[i] = Wire.read();
  return true;
}

static bool mpuReadAccelGyroRaw(int16_t &ax, int16_t &ay, int16_t &az,
                               int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t buf[14];
  if (!i2cReadBytes(MPU_ADDR, REG_ACCEL_XOUT_H, buf, 14)) return false;

  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);

  gx = (int16_t)((buf[8]  << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);
  return true;
}

static bool mpuInit() {
  if (!i2cWriteByte(MPU_ADDR, REG_PWR_MGMT_1, 0x00)) return false;
  delay(50);

  i2cWriteByte(MPU_ADDR, REG_CONFIG, 0x03);
  i2cWriteByte(MPU_ADDR, REG_SMPLRT_DIV, 0x04);

  if (!i2cWriteByte(MPU_ADDR, REG_GYRO_CONFIG, 0x00)) return false;
  delay(10);

  if (!i2cWriteByte(MPU_ADDR, REG_ACCEL_CONFIG, 0x00)) return false;
  delay(10);

  uint8_t who = 0;
  if (!i2cReadBytes(MPU_ADDR, REG_WHO_AM_I, &who, 1)) return false;
  Serial.printf("[MPU] WHO_AM_I=0x%02X\n", who);
  return true;
}

static void mpuCalibrate(uint32_t ms = 2000) {
  Serial.println("[MPU] Calibrating... keep STILL on a flat surface");

  uint32_t t0 = millis();
  uint32_t n = 0;

  double sumAx=0, sumAy=0, sumAz=0;
  double sumGx=0, sumGy=0, sumGz=0;

  while (millis() - t0 < ms) {
    int16_t rax, ray, raz, rgx, rgy, rgz;
    if (mpuReadAccelGyroRaw(rax, ray, raz, rgx, rgy, rgz)) {
      float axg = (float)rax / ACC_SENS_2G;
      float ayg = (float)ray / ACC_SENS_2G;
      float azg = (float)raz / ACC_SENS_2G;

      float gxd = (float)rgx / GYRO_SENS_250;
      float gyd = (float)rgy / GYRO_SENS_250;
      float gzd = (float)rgz / GYRO_SENS_250;

      sumAx += axg; sumAy += ayg; sumAz += azg;
      sumGx += gxd; sumGy += gyd; sumGz += gzd;
      n++;
    }
    delay(5);
  }

  if (n == 0) return;

  float meanAx = (float)(sumAx / n);
  float meanAy = (float)(sumAy / n);
  float meanAz = (float)(sumAz / n);

  gxOff = (float)(sumGx / n);
  gyOff = (float)(sumGy / n);
  gzOff = (float)(sumGz / n);

  azSign = (meanAz >= 0.0f) ? 1.0f : -1.0f;

  axOff = meanAx;
  ayOff = meanAy;
  azOff = meanAz - azSign * 1.0f;

  float ax0 = meanAx - axOff;
  float ay0 = meanAy - ayOff;
  float az0 = meanAz - azOff;

  float pitchAcc0 = atan2f( ay0, sqrtf(ax0*ax0 + az0*az0) ) * RAD2DEG;
  float rollAcc0  = atan2f( -ax0, az0 ) * RAD2DEG;

  pitchDeg = pitchAcc0;
  rollDeg  = rollAcc0;

  Serial.printf("[MPU] Cal done. mean(g): ax=%.3f ay=%.3f az=%.3f | azSign=%.0f\n",
                meanAx, meanAy, meanAz, azSign);
}

// =======================================================
//                      Modalità (3 stati)
// =======================================================
enum Mode : uint8_t { MODE_JOYSTICK=1, MODE_TILT=2, MODE_AUTO=3 };
static Mode g_mode = MODE_JOYSTICK;

// debounce SW1
static bool     g_btnPrev = false;
static uint32_t g_btnT0   = 0;
static const uint32_t BTN_DEBOUNCE_MS = 120;

static int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static int deadzone100(int v, int dz) {
  if (abs(v) <= dz) return 0;
  return (v > 0) ? (v - dz) : (v + dz);
}

static const char* dirTextFromSteer(int steer) {
  if (steer > 15)  return "RIGHT";
  if (steer < -15) return "LEFT";
  return "STRAIGHT";
}

static const char* modeText(uint8_t m) {
  if (m == MODE_TILT) return "TILT";
  if (m == MODE_AUTO) return "AUTO";
  return "JOY";
}

// ✅ FIX 2: calibrazione centro joystick (drive)
static int xCenter = 2048;
static int yCenter = 2048;

static void calibrateJoystickCenter() {
  long sx = 0, sy = 0;
  for (int i = 0; i < 200; i++) {
    sx += analogRead(PIN_VRX);
    sy += analogRead(PIN_VRY);
    delay(2);
  }
  xCenter = (int)(sx / 200);
  yCenter = (int)(sy / 200);
  Serial.printf("[JOY] center calibrated: x=%d y=%d\n", xCenter, yCenter);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_SW, INPUT_PULLUP);
  pinMode(PIN2_SW, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Calibra centro joystick DOPO init ADC
  calibrateJoystickCenter();

  Link.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println("[SENDER] UART: D,mode,btn,pitch,roll,dir");

  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (!mpuInit()) {
    Serial.println("[MPU] Init FAILED");
  } else {
    Serial.println("[MPU] Init OK");
    mpuCalibrate(2000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  esp_wifi_set_channel(ESPNOW_CH, WIFI_SECOND_CHAN_NONE);

  Serial.print("[ESP-NOW] TX STA MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("[ESP-NOW] Channel forced to %u\n", (unsigned)ESPNOW_CH);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, RX_MAC, 6);
  peer.channel = ESPNOW_CH;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESP-NOW] add_peer failed");
    while (true) delay(1000);
  }

  Serial.println("[SENDER] Ready: SW1 cycles JOY->TILT->AUTO, SW2 centers cam");
}

void loop() {
  static uint32_t lastMs = millis();
  uint32_t now = millis();
  float dt = (now - lastMs) / 1000.0f;
  if (dt <= 0.0f) dt = 0.01f;
  if (dt > 0.1f)  dt = 0.1f;
  lastMs = now;

  // ---- joystick 1 raw ----
  int xRaw = analogRead(PIN_VRX);
  int yRaw = analogRead(PIN_VRY);
  bool btnPressed = (digitalRead(PIN_SW) == LOW);

  // ---- modalità su pressione (debounce) ----
  if (btnPressed != g_btnPrev) { g_btnPrev = btnPressed; g_btnT0 = now; }
  if ((now - g_btnT0) > BTN_DEBOUNCE_MS) {
    static bool lastStable = false;
    if (btnPressed != lastStable) {
      lastStable = btnPressed;
      if (lastStable) {
        if (g_mode == MODE_JOYSTICK) g_mode = MODE_TILT;
        else if (g_mode == MODE_TILT) g_mode = MODE_AUTO;
        else g_mode = MODE_JOYSTICK;

        Serial.print("[MODE] -> ");
        Serial.println(modeText(g_mode));
      }
    }
  }

  // ---- joystick 2 (PAN) ----
  int camRawX = analogRead(PIN2_VRX);
  int camMappedX = map(camRawX, 0, 4095, -100, 100);
  camMappedX = deadzone100(camMappedX, 8);

  bool camCenterPressed = (digitalRead(PIN2_SW) == LOW);
  if (camCenterPressed) camMappedX = 0;

  // ---- IMU read + complementary filter ----
  int16_t rax=0, ray=0, raz=0, rgx=0, rgy=0, rgz=0;
  bool imuOk = mpuReadAccelGyroRaw(rax, ray, raz, rgx, rgy, rgz);

  float pitchNow = pitchDeg;
  float rollNow  = rollDeg;

  if (imuOk) {
    float axg = (float)rax / ACC_SENS_2G - axOff;
    float ayg = (float)ray / ACC_SENS_2G - ayOff;
    float azg = (float)raz / ACC_SENS_2G - azOff;

    float gxd = (float)rgx / GYRO_SENS_250 - gxOff;
    float gyd = (float)rgy / GYRO_SENS_250 - gyOff;
    (void)rgz;

    float pitchAcc = atan2f( ayg, sqrtf(axg*axg + azg*azg) ) * RAD2DEG;
    float rollAcc  = atan2f( -axg, azg ) * RAD2DEG;

    float pitchGyro = pitchDeg + (gxd * dt);
    float rollGyro  = rollDeg  + (gyd * dt);

    pitchDeg = ALPHA * pitchGyro + (1.0f - ALPHA) * pitchAcc;
    rollDeg  = ALPHA * rollGyro  + (1.0f - ALPHA) * rollAcc;

    pitchNow = pitchDeg;
    rollNow  = rollDeg;
  }

  // ---- joystick 1 map (relative to calibrated center) ----
  int xRel = xRaw - xCenter;
  int yRel = yRaw - yCenter;

  int xMapped = map(xRel, -2048, 2047, -100, 100);
  int yMapped = map(yRel, -2048, 2047, -100, 100);

  // deadzone drive
  xMapped = deadzone100(xMapped, 8);
  yMapped = deadzone100(yMapped, 8);

  // ---- choose outputs based on mode ----
  const int SCALE = 162;
  bool autoMode = (g_mode == MODE_AUTO);

  int steer = 0;
  int outAx = 0;
  int outAy = 0;

  if (g_mode == MODE_JOYSTICK) {
    outAx = xMapped;
    outAy = yMapped;

    // ✅ FIX 1: clamp throttle a 0 quando vuoi pivot
    const int PIVOT_STEER = 25;
    const int PIVOT_THR   = 20;
    if (abs(outAx) > PIVOT_STEER && abs(outAy) < PIVOT_THR) {
      outAy = 0;
    }

    steer = outAx;
  } else if (g_mode == MODE_TILT) {
    const float PITCH_MAX_DEG = 25.0f;
    const float ROLL_MAX_DEG  = 25.0f;

    int pitchCmd = (int)lroundf((pitchNow / PITCH_MAX_DEG) * 100.0f);
    int rollCmd  = (int)lroundf((rollNow  / ROLL_MAX_DEG)  * 100.0f);

    pitchCmd = clampInt(pitchCmd, -100, 100);
    rollCmd  = clampInt(rollCmd,  -100, 100);

    pitchCmd = deadzone100(pitchCmd, 6);
    rollCmd  = deadzone100(rollCmd,  6);

    outAx = rollCmd;
    outAy = pitchCmd;
    steer = rollCmd;
  } else {
    outAx = 0;
    outAy = 0;
    steer = 0;
  }

  int outAz = camMappedX;
  const char* dirTxt = dirTextFromSteer(steer);

  Link.printf("D,%d,%d,%.1f,%.1f,%s\n",
              (int)g_mode, (btnPressed ? 1 : 0), pitchNow, rollNow, dirTxt);

  // ---- send 20 Hz ----
  static uint32_t lastSendMs = 0;
  if (now - lastSendMs >= 50) {
    lastSendMs = now;

    ControlMsg msg;
    msg.autoMode = autoMode;
    msg.ax = (int16_t)(outAx * SCALE);
    msg.ay = (int16_t)(outAy * SCALE);
    msg.az = (int16_t)(outAz * SCALE);

    esp_now_send(RX_MAC, (uint8_t*)&msg, sizeof(msg));

    Serial.printf("[TX] mode=%d auto=%d | ax=%d ay=%d az=%d | pitch=%.1f roll=%.1f dir=%s imu=%s | joyCenter=(%d,%d)\n",
                  (int)g_mode, (int)msg.autoMode,
                  (int)msg.ax, (int)msg.ay, (int)msg.az,
                  pitchNow, rollNow, dirTxt, imuOk ? "OK" : "FAIL",
                  xCenter, yCenter);
  }

  delay(5);
}
