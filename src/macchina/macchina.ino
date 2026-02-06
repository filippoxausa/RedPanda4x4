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
  int16_t ax, ay, az; // az = camera joystick2 (pan)
} ControlMsg;

struct MotorCmd { int16_t left; int16_t right; };

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
  s_ax = msg.ax; s_ay = msg.ay; s_az = msg.az;
  s_lastRxMs = millis();
}
#else
void EspNowReceiver::onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
  if (len != (int)sizeof(ControlMsg)) return;
  ControlMsg msg;
  memcpy(&msg, data, sizeof(msg));
  s_ax = msg.ax; s_ay = msg.ay; s_az = msg.az;
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
  out.ax = s_ax; out.ay = s_ay; out.az = s_az;
  ageMs = millis() - (uint32_t)s_lastRxMs;
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
static const int CH_A = 0;
static const int CH_B = 1;

static void motorEnable(bool en) { digitalWrite(PIN_STBY, en ? HIGH : LOW); }

static void driveMotor(int in1, int in2, int pwmCh, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWriteChannel(pwmCh, (uint32_t)speed);
#else
    ledcWrite(pwmCh, speed);
#endif
  } else if (speed < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWriteChannel(pwmCh, (uint32_t)(-speed));
#else
    ledcWrite(pwmCh, -speed);
#endif
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
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
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(PIN_PWMA, PWM_FREQ, PWM_RES, CH_A);
  ledcAttachChannel(PIN_PWMB, PWM_FREQ, PWM_RES, CH_B);
#else
  ledcSetup(CH_A, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_PWMA, CH_A);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_PWMB, CH_B);
#endif

  motorEnable(true);
  driveLeft(0); driveRight(0);
}

static void motorsSet(int16_t left, int16_t right) { driveLeft(left); driveRight(right); }
static void motorsStop() { motorsSet(0, 0); }

// =======================================================
//                 Helpers (clamp/map/deadzone)
// =======================================================
static inline int clampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

static inline int mapInt(int x, int inMin, int inMax, int outMin, int outMax) {
  long num = (long)(x - inMin) * (outMax - outMin);
  long den = (inMax - inMin);
  return (int)(outMin + num / den);
}

static inline int applyDeadzone(int v, int dz) {
  if (abs(v) < dz) return 0;
  return (v > 0) ? (v - dz) : (v + dz);
}

// =======================================================
//                   CONTROL -> MOTORS (MIXING)
// =======================================================
static const int AX_AY_MAX = 16200;
static const int DEADZONE  = 600;
static const uint32_t RX_TIMEOUT_MS = 250;

static MotorCmd accelToMotors(const ControlMsg &a, bool blockReverse) {
  int steer = clampInt((int)a.ax, -AX_AY_MAX, AX_AY_MAX);
  int thr   = clampInt((int)a.ay, -AX_AY_MAX, AX_AY_MAX);

  steer = applyDeadzone(steer, DEADZONE);
  thr   = applyDeadzone(thr,   DEADZONE);

  int steerPWM = mapInt(steer, -AX_AY_MAX, AX_AY_MAX, -255, 255);
  int thrPWM   = mapInt(thr,   -AX_AY_MAX, AX_AY_MAX, -255, 255);

  // blocca solo retro
  if (blockReverse && thrPWM < 0) thrPWM = 0;

  int left  = clampInt(thrPWM + steerPWM, -255, 255);
  int right = clampInt(thrPWM - steerPWM, -255, 255);

  MotorCmd m;
  m.left  = (int16_t)left;
  m.right = (int16_t)right;
  return m;
}

// =======================================================
//           IR reverse sensor (DIGITAL obstacle)
// =======================================================
static const int PIN_IR_BACK = 23;
static const bool IR_ACTIVE_LOW = true;

static void irInit() {
  pinMode(PIN_IR_BACK, IR_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
}

static bool irObstacle() {
  int v = digitalRead(PIN_IR_BACK);
  return IR_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

// =======================================================
//                    SERVO + HC-SR04
// =======================================================
static const int PIN_SERVO_US  = 33;
static const int PIN_SERVO_CAM = 32;

static const int PIN_TRIG = 18;
static const int PIN_ECHO = 19;

static const int SERVO_FREQ = 50;
static const int SERVO_RES  = 16;
static const int CH_SERVO_US  = 2;
static const int CH_SERVO_CAM = 3;

// SG90 safe
static const int SERVO_MIN_US = 650;
static const int SERVO_MAX_US = 2350;

// front-only (anti blocco)
static const int SERVO_FWD_MIN_ANG = 60;
static const int SERVO_FWD_MAX_ANG = 120;

static inline uint32_t servoDutyFromUs(int pulseUs) {
  const int periodUs = 20000;
  pulseUs = clampInt(pulseUs, SERVO_MIN_US, SERVO_MAX_US);
  const uint32_t maxDuty = (1UL << SERVO_RES) - 1;
  return (uint32_t)((uint64_t)pulseUs * maxDuty / periodUs);
}

static void servoWriteAngle(int ch, int angleDeg) {
  angleDeg = clampInt(angleDeg, SERVO_FWD_MIN_ANG, SERVO_FWD_MAX_ANG);
  int pulseUs = mapInt(angleDeg, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  uint32_t duty = servoDutyFromUs(pulseUs);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(ch, duty);
#else
  ledcWrite(ch, duty);
#endif
}

static void servosInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(PIN_SERVO_US,  SERVO_FREQ, SERVO_RES, CH_SERVO_US);
  ledcAttachChannel(PIN_SERVO_CAM, SERVO_FREQ, SERVO_RES, CH_SERVO_CAM);
#else
  ledcSetup(CH_SERVO_US, SERVO_FREQ, SERVO_RES);  ledcAttachPin(PIN_SERVO_US, CH_SERVO_US);
  ledcSetup(CH_SERVO_CAM, SERVO_FREQ, SERVO_RES); ledcAttachPin(PIN_SERVO_CAM, CH_SERVO_CAM);
#endif
  servoWriteAngle(CH_SERVO_US,  90);
  servoWriteAngle(CH_SERVO_CAM, 90);
}

static void hcsr04Init() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
}

static uint16_t hcsr04ReadCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  uint32_t us = pulseIn(PIN_ECHO, HIGH, 25000);
  if (us == 0) return 0;
  return (uint16_t)(us / 58);
}

// sweep ultrasuoni
static int usAngle = 90;
static int usDir = +1;
static const uint32_t SERVO_SWEEP_PERIOD_MS = 40;

// =======================================================
//                         MAIN
// =======================================================
static EspNowReceiver rx;

void setup() {
  Serial.begin(115200);
  delay(200);

  motorsInit();
  servosInit();
  hcsr04Init();
  irInit();

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

  const uint32_t now = millis();

  // failsafe radio
  if (ageMs > RX_TIMEOUT_MS) {
    motorsStop();
    servoWriteAngle(CH_SERVO_US, 90);
    servoWriteAngle(CH_SERVO_CAM, 90);
    delay(5);
    return;
  }

  // IR retro
  bool backObs = irObstacle();
  bool wantReverse = ((int)msg.ay < -DEADZONE);

  // motori (blocca solo retro se ostacolo)
  MotorCmd m = accelToMotors(msg, wantReverse && backObs);
  motorsSet(m.left, m.right);

  // servo camera: az in [-16200..+16200] -> angolo 0..180 (poi clamp front-only)
  int camAngle = mapInt(clampInt((int)msg.az, -AX_AY_MAX, AX_AY_MAX),
                        -AX_AY_MAX, AX_AY_MAX, 0, 180);
  servoWriteAngle(CH_SERVO_CAM, camAngle);

  // sweep ultrasuoni
  static uint32_t tServo = 0;
  if (now - tServo >= SERVO_SWEEP_PERIOD_MS) {
    tServo = now;
    usAngle += usDir * 2;
    if (usAngle >= SERVO_FWD_MAX_ANG) { usAngle = SERVO_FWD_MAX_ANG; usDir = -1; }
    if (usAngle <= SERVO_FWD_MIN_ANG) { usAngle = SERVO_FWD_MIN_ANG; usDir = +1; }
    servoWriteAngle(CH_SERVO_US, usAngle);
  }

  // ultrasuoni ogni 100ms + stampa ogni 200ms
  static uint32_t tUS = 0, tPrint = 0;
  static uint16_t distCm = 0;

  if (now - tUS >= 100) {
    tUS = now;
    distCm = hcsr04ReadCm();
  }

  if (now - tPrint >= 200) {
    tPrint = now;
    Serial.print("ax="); Serial.print(msg.ax);
    Serial.print(" ay="); Serial.print(msg.ay);
    Serial.print(" az(cam)="); Serial.print(msg.az);
    Serial.print(" camAng="); Serial.print(camAngle);
    Serial.print(" distCm="); Serial.print(distCm);
    Serial.print(" IR_back="); Serial.println(backObs ? 1 : 0);
  }

  delay(5);
}
