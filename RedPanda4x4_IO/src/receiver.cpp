#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ================== PIN TB6612 (i tuoi) ==================
static const int PIN_AIN1 = 14;
static const int PIN_AIN2 = 27;
static const int PIN_PWMA = 26;

static const int PIN_BIN1 = 16;
static const int PIN_BIN2 = 17;
static const int PIN_PWMB = 4;

static const int PIN_STBY = 25;

// ================== PWM (LEDC) ==================
static const int PWM_FREQ = 20000;
static const int PWM_RES  = 8;
static const int CH_A = 0;
static const int CH_B = 1;

// ====== Protocollo messaggio ======
typedef struct __attribute__((packed)) {
  int16_t speed; // -255..+255
} ControlMsg;

static volatile int16_t  g_speed = 0;
static volatile uint32_t g_lastRxMs = 0;

static void motorEnable(bool en) {
  digitalWrite(PIN_STBY, en ? HIGH : LOW);
}

static void driveMotor(int in1, int in2, int pwmCh, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmCh, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmCh, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmCh, 0);
  }
}

static void driveA(int speed) { driveMotor(PIN_AIN1, PIN_AIN2, CH_A, speed); }
static void driveB(int speed) { driveMotor(PIN_BIN1, PIN_BIN2, CH_B, speed); }

// Callback RX: firma "vecchia" (compatibile con molte versioni Arduino-ESP32) [web:69]
static void onRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  (void)mac_addr; // non usato

  if (len != (int)sizeof(ControlMsg)) return;

  ControlMsg msg;
  memcpy(&msg, data, sizeof(msg));
  g_speed = msg.speed;
  g_lastRxMs = millis();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // TB6612 init
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  ledcSetup(CH_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWMA, CH_A);

  ledcSetup(CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWMB, CH_B);

  motorEnable(true);
  driveA(0);
  driveB(0);

  // ESP-NOW init
  WiFi.mode(WIFI_STA);
  Serial.print("RX STA MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onRecv);
  g_lastRxMs = millis();
}

void loop() {
  const uint32_t now = millis();

  // Fail-safe: se non riceve pacchetti per 300 ms -> STOP
  if (now - g_lastRxMs > 300) {
    driveA(0);
    driveB(0);
  } else {
    int16_t s = g_speed;
    driveA(s);
    driveB(s);
  }

  static uint32_t t0 = 0;
  if (now - t0 > 200) {
    t0 = now;
    Serial.print("RX speed=");
    Serial.println((int)g_speed);
  }

  delay(5);
}
