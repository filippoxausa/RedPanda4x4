#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <string.h>

#include <esp_arduino_version.h>
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #include <esp_wifi.h>
#endif

// =======================================================
//                    CONTROL MSG (packed)
// =======================================================
typedef struct __attribute__((packed)) {
  int16_t ax, ay, az;
} ControlMsg;

// =======================================================
//                   MOTOR COMMAND TYPE
// =======================================================
struct MotorCmd {
  int16_t left;
  int16_t right;
};

// =======================================================
//                       ESPNOW RX
// =======================================================
class EspNowReceiver {
public:
  bool begin();
  void getLatest(ControlMsg &out, uint32_t &ageMs) const;

private:
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
#else
  static void onRecv(const uint8_t *mac, const uint8_t *data, int len);
#endif

  static volatile int16_t  s_ax, s_ay, s_az;
  static volatile uint32_t s_lastRxMs;
};

volatile int16_t  EspNowReceiver::s_ax = 0;
volatile int16_t  EspNowReceiver::s_ay = 0;
volatile int16_t  EspNowReceiver::s_az = 0;
volatile uint32_t EspNowReceiver::s_lastRxMs = 0;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void EspNowReceiver::onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  if (len != (int)sizeof(ControlMsg)) return;

  ControlMsg msg;
  memcpy(&msg, data, sizeof(msg));

  s_ax = msg.ax;
  s_ay = msg.ay;
  s_az = msg.az;
  s_lastRxMs = millis();
}
#else
void EspNowReceiver::onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
  if (len != (int)sizeof(ControlMsg)) return;

  ControlMsg msg;
  memcpy(&msg, data, sizeof(msg));

  s_ax = msg.ax;
  s_ay = msg.ay;
  s_az = msg.az;
  s_lastRxMs = millis();
}
#endif

bool EspNowReceiver::begin() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(EspNowReceiver::onRecv);

  s_lastRxMs = millis();
  return true;
}

void EspNowReceiver::getLatest(ControlMsg &out, uint32_t &ageMs) const {
  out.ax = s_ax;
  out.ay = s_ay;
  out.az = s_az;

  uint32_t now = millis();
  uint32_t last = s_lastRxMs;
  ageMs = now - last;
}

// =======================================================
//                    MOTOR TB6612FNG
// =======================================================
static const int PIN_AIN1 = 14;
static const int PIN_AIN2 = 27;
static const int PIN_PWMA = 26;

static const int PIN_BIN1 = 16;
static const int PIN_BIN2 = 17;
static const int PIN_PWMB = 4;

static const int PIN_STBY = 25;

static const int PWM_FREQ = 20000;
static const int PWM_RES  = 8;
static const int CH_A = 0; // left
static const int CH_B = 1; // right

static void motorEnable(bool en) {
  digitalWrite(PIN_STBY, en ? HIGH : LOW);
}

static void driveMotor(int in1, int in2, int pwmCh, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWriteChannel(pwmCh, (uint32_t)speed);
#else
    ledcWrite(pwmCh, speed);
#endif
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWriteChannel(pwmCh, (uint32_t)(-speed));
#else
    ledcWrite(pwmCh, -speed);
#endif
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWriteChannel(pwmCh, 0);
#else
    ledcWrite(pwmCh, 0);
#endif
  }
}

static void driveLeft(int speed)  { driveMotor(PIN_AIN1, PIN_AIN2, CH_A, speed); }
static void driveRight(int speed) { driveMotor(PIN_BIN1, PIN_BIN2, CH_B, speed); }

static void motorsInit() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(PIN_PWMA, PWM_FREQ, PWM_RES, CH_A);
  ledcAttachChannel(PIN_PWMB, PWM_FREQ, PWM_RES, CH_B);
#else
  ledcSetup(CH_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWMA, CH_A);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWMB, CH_B);
#endif

  motorEnable(true);
  driveLeft(0);
  driveRight(0);
}

static void motorsSet(int16_t left, int16_t right) {
  driveLeft(left);
  driveRight(right);
}

static void motorsStop() {
  motorsSet(0, 0);
}

// =======================================================
//                   CONTROL -> MOTORS (MIXING)
// =======================================================
static const int AX_AY_MAX = 16200;     // sender: [-100..100]*162
static const int DEADZONE  = 600;       // elimina jitter
static const uint32_t RX_TIMEOUT_MS = 250;

static inline int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int mapInt(int x, int inMin, int inMax, int outMin, int outMax) {
  long num = (long)(x - inMin) * (outMax - outMin);
  long den = (inMax - inMin);
  return (int)(outMin + num / den);
}

static inline int applyDeadzone(int v, int dz) {
  if (abs(v) < dz) return 0;
  if (v > 0) return v - dz;
  return v + dz;
}

static MotorCmd accelToMotors(const ControlMsg &a) {
  int steer = clampInt((int)a.ax, -AX_AY_MAX, AX_AY_MAX);
  int thr   = clampInt((int)a.ay, -AX_AY_MAX, AX_AY_MAX);

  steer = applyDeadzone(steer, DEADZONE);
  thr   = applyDeadzone(thr,   DEADZONE);

  int steerPWM = mapInt(steer, -AX_AY_MAX, AX_AY_MAX, -255, 255);
  int thrPWM   = mapInt(thr,   -AX_AY_MAX, AX_AY_MAX, -255, 255);

  // Se va al contrario quando spingi avanti, inverti thrPWM:
  // thrPWM = -thrPWM;

  int left  = clampInt(thrPWM + steerPWM, -255, 255);
  int right = clampInt(thrPWM - steerPWM, -255, 255);

  MotorCmd m;
  m.left  = (int16_t)left;
  m.right = (int16_t)right;
  return m;
}

// =======================================================
//                         MAIN
// =======================================================
static EspNowReceiver rx;

void setup() {
  motorsInit();

  if (!rx.begin()) {
    motorsStop();
    while (true) delay(1000);
  }

  motorsStop();
}

void loop() {
  ControlMsg msg;
  uint32_t ageMs;
  rx.getLatest(msg, ageMs);

  if (ageMs > RX_TIMEOUT_MS) {
    motorsStop();
    delay(5);
    return;
  }

  MotorCmd m = accelToMotors(msg);
  motorsSet(m.left, m.right);

  delay(5);
}
