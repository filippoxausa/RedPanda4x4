#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <math.h>

// =======================================================
//                    UART -> CYD (monitor)
// =======================================================
static const int UART_TX = 17;   // TX sender -> RX CYD (GPIO27)
static const int UART_RX = -1;   // non usato
HardwareSerial Link(2);

// =======================================================
//                       JOYSTICK
// =======================================================
static const int PIN_VRX = 34;   // ADC1
static const int PIN_VRY = 35;   // ADC1
static const int PIN_SW  = 33;   // button (LOW = premuto)

// =======================================================
//                      MPU6500 (I2C)
// =======================================================
static const uint8_t MPU_ADDR = 0x68; // AD0=GND
static const uint8_t REG_WHO_AM_I    = 0x75;
static const uint8_t REG_PWR_MGMT_1  = 0x6B;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_GYRO_ZOUT_H = 0x47;

static const float GYRO_SENS_250 = 131.0f; // LSB/(deg/s) @ ±250 dps
static float headingDeg = 0.0f;

// =======================================================
//                       ESP-NOW
// =======================================================
static const uint8_t RX_MAC[6] = { 0xE0,0x8C,0xFE,0x2E,0x96,0x7C };

// Struct IDENTICA a quella del receiver
typedef struct __attribute__((packed)) {
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

static bool mpuInit() {
  if (!i2cWriteByte(MPU_ADDR, REG_PWR_MGMT_1, 0x00)) return false; // wake
  delay(50);
  if (!i2cWriteByte(MPU_ADDR, REG_GYRO_CONFIG, 0x00)) return false; // ±250 dps
  delay(10);

  uint8_t who = 0;
  if (!i2cReadBytes(MPU_ADDR, REG_WHO_AM_I, &who, 1)) return false;
  Serial.printf("[MPU] WHO_AM_I=0x%02X\n", who);
  return true;
}

static bool readGyroZ(int16_t &gzRaw) {
  uint8_t buf[2];
  if (!i2cReadBytes(MPU_ADDR, REG_GYRO_ZOUT_H, buf, 2)) return false;
  gzRaw = (int16_t)((buf[0] << 8) | buf[1]);
  return true;
}

// =======================================================
//                      Modalità
// =======================================================
enum Mode : uint8_t {
  MODE_JOYSTICK = 1,
  MODE_GYRO     = 2
};

static Mode g_mode = MODE_JOYSTICK;

// Debounce button (toggle su fronte)
static bool     g_btnPrev = false;
static uint32_t g_btnT0   = 0;
static const uint32_t BTN_DEBOUNCE_MS = 120;

// =======================================================
//                 Helpers mappa e direzione
// =======================================================
static int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// direzione testuale in base a uno "steer" (-100..100)
static const char* dirTextFromSteer(int steer) {
  if (steer > 15)  return "RIGHT";
  if (steer < -15) return "LEFT";
  return "STRAIGHT";
}

// =======================================================
//                         SETUP
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  // Joystick ADC
  pinMode(PIN_SW, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // UART2 -> CYD (monitor)
  Link.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println("[SENDER] UART2->CYD: D,mode,btn,yaw,head,dir");

  // I2C + MPU
  Wire.begin(21, 22);
  Wire.setClock(400000);
  if (!mpuInit()) Serial.println("[MPU] Init FAILED");
  else            Serial.println("[MPU] Init OK");

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  Serial.print("[ESP-NOW] TX STA MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Error initializing ESP-NOW");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, RX_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESP-NOW] esp_now_add_peer failed");
    while (true) delay(1000);
  }

  Serial.println("[SENDER] Ready: MODE toggle on joystick button");
}

// =======================================================
//                          LOOP
// =======================================================
void loop() {
  static uint32_t lastMs = millis();
  uint32_t now = millis();
  float dt = (now - lastMs) / 1000.0f;
  if (dt <= 0) dt = 0.02f;
  lastMs = now;

  // ---- Leggi joystick ----
  int xRaw = analogRead(PIN_VRX);
  int yRaw = analogRead(PIN_VRY);
  bool btnPressed = (digitalRead(PIN_SW) == LOW);

  // ---- Toggle modalità su fronte (debounce) ----
  if (btnPressed != g_btnPrev) {
    g_btnPrev = btnPressed;
    g_btnT0 = now;
  }
  // se stabile e appena premuto (LOW) -> toggle
  if ((now - g_btnT0) > BTN_DEBOUNCE_MS) {
    static bool lastStable = false;
    if (btnPressed != lastStable) {
      lastStable = btnPressed;
      if (lastStable) { // transizione a premuto
        g_mode = (g_mode == MODE_JOYSTICK) ? MODE_GYRO : MODE_JOYSTICK;
        Serial.printf("[MODE] -> %s\n", (g_mode == MODE_JOYSTICK) ? "JOYSTICK" : "GYRO");
      }
    }
  }

  int btn = btnPressed ? 1 : 0;

  // ---- Gyro Z -> yaw + heading ----
  int16_t gzRaw = 0;
  float yawDps = 0.0f;

  if (readGyroZ(gzRaw)) {
    yawDps = (float)gzRaw / GYRO_SENS_250;
    if (fabsf(yawDps) < 1.5f) yawDps = 0.0f; // deadband gyro

    headingDeg += yawDps * dt;
    if (headingDeg > 180.0f) headingDeg -= 360.0f;
    if (headingDeg < -180.0f) headingDeg += 360.0f;
  } else {
    yawDps = 0.0f;
  }

  // ---- Mappa joystick come sul monitor (-100..100) ----
  int xMapped = map(xRaw, 0, 4095, -100, 100);
  int yMapped = map(yRaw, 0, 4095, -100, 100);

  // ---- Scegli cosa inviare via ESP-NOW in base alla modalità ----
  // Manteniamo il tuo scaling *162 (come nel codice attuale)
  const int SCALE = 162;

  int steer = 0;     // -100..100 (per direzione testo)
  int outAx = 0;
  int outAy = 0;

  if (g_mode == MODE_JOYSTICK) {
    // Mode 1: joystick -> ax, ay
    steer = xMapped;
    outAx = xMapped;
    outAy = yMapped;
  } else {
    // Mode 2: gyro -> sterzo, joystick Y -> throttle
    // Converti yawDps in range -100..100 (regola a gusto)
    // Con 120 dps saturi a 100.
    const float YAW_MAX_DPS = 120.0f;
    int yawMapped = (int)lroundf((yawDps / YAW_MAX_DPS) * 100.0f);
    yawMapped = clampInt(yawMapped, -100, 100);

    steer = yawMapped;
    outAx = yawMapped;  // ax = "sterzo da gyro"
    outAy = yMapped;    // ay = avanti/indietro da joystick
  }

  const char* dirTxt = dirTextFromSteer(steer);

  // ---- UART al monitor (nuovo formato) ----
  // D,mode,btn,yaw,head,dir
  Link.printf("D,%d,%d,%.1f,%.1f,%s\n",
              (int)g_mode, btn, yawDps, headingDeg, dirTxt);

  // ---- ESP-NOW send a 20 Hz ----
  static uint32_t lastSendMs = 0;
  const uint32_t SEND_PERIOD_MS = 50;

  if (now - lastSendMs >= SEND_PERIOD_MS) {
    lastSendMs = now;

    ControlMsg msg;
    msg.ax = (int16_t)(outAx * SCALE);
    msg.ay = (int16_t)(outAy * SCALE);
    msg.az = 0;

    esp_err_t err = esp_now_send(RX_MAC, (uint8_t*)&msg, sizeof(msg));
    if (err != ESP_OK) {
      Serial.printf("[ESP-NOW] send error=%d\n", (int)err);
    }

    Serial.printf("[TX] mode=%d btn=%d | outAx=%d outAy=%d (scaled ax=%d ay=%d) | yaw=%.1f head=%.1f dir=%s\n",
                  (int)g_mode, btn, outAx, outAy, (int)msg.ax, (int)msg.ay, yawDps, headingDeg, dirTxt);
  }

  delay(5);
}
