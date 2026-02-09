#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <string.h>

#include <esp_arduino_version.h>
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #include <esp_wifi.h>
#endif

// =======================================================
//                QUICK FIX SWITCHES (CALIBRAZIONE)
// =======================================================
// Se in manuale avanti va indietro -> metti INVERT_THROTTLE_AXIS = true
static const bool INVERT_THROTTLE_AXIS = true;   // <-- PROVA true/false
// Se sterzo è invertito (dx/sx) -> metti INVERT_STEER_AXIS = true
static const bool INVERT_STEER_AXIS    = false;  // <-- PROVA true/false

// Se i motori fisicamente vanno al contrario -> inverti il lato
static const bool INVERT_LEFT_MOTOR    = false;  // <-- PROVA true/false
static const bool INVERT_RIGHT_MOTOR   = true;   // <-- PROVA true/false

// =======================================================
//                    CONTROL MSG (packed)
// =======================================================
typedef struct __attribute__((packed)) {
  bool    autoMode;
  int16_t ax, ay, az; // manual: steer/throttle/camPan (scaled)
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

  static volatile bool     s_auto;
  static volatile int16_t  s_ax, s_ay, s_az;
  static volatile uint32_t s_lastRxMs;
};

volatile bool     EspNowReceiver::s_auto = false;
volatile int16_t  EspNowReceiver::s_ax   = 0;
volatile int16_t  EspNowReceiver::s_ay   = 0;
volatile int16_t  EspNowReceiver::s_az   = 0;
volatile uint32_t EspNowReceiver::s_lastRxMs = 0;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void EspNowReceiver::onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  if (len != (int)sizeof(ControlMsg)) return;
  ControlMsg msg;
  memcpy(&msg, data, sizeof(msg));
  s_auto = msg.autoMode;
  s_ax = msg.ax; s_ay = msg.ay; s_az = msg.az;
  s_lastRxMs = millis();
}
#else
void EspNowReceiver::onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
  if (len != (int)sizeof(ControlMsg)) return;
  ControlMsg msg;
  memcpy(&msg, data, sizeof(msg));
  s_auto = msg.autoMode;
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
  out.autoMode = s_auto;
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
static const int CH_A = 0; // left
static const int CH_B = 1; // right

static void motorEnable(bool en) { digitalWrite(PIN_STBY, en ? HIGH : LOW); }

static void ledcWriteCompat(int ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(ch, duty);
#else
  ledcWrite(ch, duty);
#endif
}

static void driveMotor(int in1, int in2, int pwmCh, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    ledcWriteCompat(pwmCh, (uint32_t)speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    ledcWriteCompat(pwmCh, (uint32_t)(-speed));
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    ledcWriteCompat(pwmCh, 0);
  }
}

static void driveLeft(int speed)  { if (INVERT_LEFT_MOTOR) speed = -speed; driveMotor(PIN_AIN1, PIN_AIN2, CH_A, speed); }
static void driveRight(int speed) { if (INVERT_RIGHT_MOTOR) speed = -speed; driveMotor(PIN_BIN1, PIN_BIN2, CH_B, speed); }

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

static void motorsRaw(int16_t left, int16_t right) {
  driveLeft(left);
  driveRight(right);
}

// anti-slip / rampa
static int16_t g_curL = 0, g_curR = 0;
static const int16_t RAMP_STEP = 14;
static const int16_t DIFF_MAX  = 230;

static int16_t stepToward(int16_t cur, int16_t tgt, int16_t step) {
  if (tgt > cur) return (int16_t)min<int>(tgt, cur + step);
  if (tgt < cur) return (int16_t)max<int>(tgt, cur - step);
  return cur;
}

static void motorsSetSmooth(int16_t targetL, int16_t targetR) {
  targetL = constrain(targetL, -255, 255);
  targetR = constrain(targetR, -255, 255);

  int diff = targetL - targetR;
  if (diff > DIFF_MAX) {
    int mid = (targetL + targetR) / 2;
    targetL = mid + DIFF_MAX/2;
    targetR = mid - DIFF_MAX/2;
  } else if (diff < -DIFF_MAX) {
    int mid = (targetL + targetR) / 2;
    targetL = mid - DIFF_MAX/2;
    targetR = mid + DIFF_MAX/2;
  }

  g_curL = stepToward(g_curL, targetL, RAMP_STEP);
  g_curR = stepToward(g_curR, targetR, RAMP_STEP);
  motorsRaw(g_curL, g_curR);
}

static void motorsStop() { motorsSetSmooth(0, 0); }

// =======================================================
//                 Helpers
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
//                MANUAL: joystick -> motors
// =======================================================
static const int AX_AY_MAX = 16200;
static const int DEADZONE  = 300;

// ✅ FIX 3: snap to zero del throttle in PWM (robusto contro drift)
static const int THR_SNAP_PWM = 35;

static MotorCmd accelToMotorsManual(ControlMsg a) {
  int steer = clampInt((int)a.ax, -AX_AY_MAX, AX_AY_MAX);
  int thr   = clampInt((int)a.ay, -AX_AY_MAX, AX_AY_MAX);

  if (INVERT_STEER_AXIS) steer = -steer;
  if (INVERT_THROTTLE_AXIS) thr = -thr;

  steer = applyDeadzone(steer, DEADZONE);
  thr   = applyDeadzone(thr,   DEADZONE);

  int steerPWM = mapInt(steer, -AX_AY_MAX, AX_AY_MAX, -255, 255);
  int thrPWM   = mapInt(thr,   -AX_AY_MAX, AX_AY_MAX, -255, 255);

  // ✅ snap-to-zero
  if (abs(thrPWM) < THR_SNAP_PWM) thrPWM = 0;

  int left = 0, right = 0;

  // ✅ wantSpin più robusto: deve essere davvero fermo per pivotare
  const bool wantSpin = (thrPWM == 0) && (abs(steerPWM) > 60);

  if (wantSpin) {
    int spin = clampInt(steerPWM / 2, -255, 255);
    left  = spin;
    right = -spin;
  } else {
    int steerScaled = (steerPWM * abs(thrPWM)) / 255;
    left  = clampInt(thrPWM + steerScaled, -255, 255);
    right = clampInt(thrPWM - steerScaled, -255, 255);
  }

  return MotorCmd{ (int16_t)left, (int16_t)right };
}

// =======================================================
//           IR reverse sensor (DIGITAL obstacle)
// =======================================================
static const int  PIN_IR_BACK = 23;
static const bool IR_ACTIVE_LOW = true;

static void irInit() {
  pinMode(PIN_IR_BACK, IR_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
}
static bool irBackObstacle() {
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

static const int SERVO_MIN_US = 650;
static const int SERVO_MAX_US = 2350;

static int SERVO_US_MIN_ANG = 55;
static int SERVO_US_MAX_ANG = 140;

static int SERVO_CAM_MIN_ANG = 35;
static int SERVO_CAM_MAX_ANG = 145;

static inline uint32_t servoDutyFromUs(int pulseUs) {
  const int periodUs = 20000;
  pulseUs = clampInt(pulseUs, SERVO_MIN_US, SERVO_MAX_US);
  const uint32_t maxDuty = (1UL << SERVO_RES) - 1;
  return (uint32_t)((uint64_t)pulseUs * maxDuty / periodUs);
}

static void servoWriteAngleRaw(int ch, int angleDeg) {
  int pulseUs = mapInt(angleDeg, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  uint32_t duty = servoDutyFromUs(pulseUs);
  ledcWriteCompat(ch, duty);
}

static void servoUS_Write(int ang) {
  ang = clampInt(ang, SERVO_US_MIN_ANG, SERVO_US_MAX_ANG);
  servoWriteAngleRaw(CH_SERVO_US, ang);
}

static void servoCam_Write(int ang) {
  ang = clampInt(ang, SERVO_CAM_MIN_ANG, SERVO_CAM_MAX_ANG);
  servoWriteAngleRaw(CH_SERVO_CAM, ang);
}

static void servosInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(PIN_SERVO_US,  SERVO_FREQ, SERVO_RES, CH_SERVO_US);
  ledcAttachChannel(PIN_SERVO_CAM, SERVO_FREQ, SERVO_RES, CH_SERVO_CAM);
#else
  ledcSetup(CH_SERVO_US, SERVO_FREQ, SERVO_RES);  ledcAttachPin(PIN_SERVO_US, CH_SERVO_US);
  ledcSetup(CH_SERVO_CAM, SERVO_FREQ, SERVO_RES); ledcAttachPin(PIN_SERVO_CAM, CH_SERVO_CAM);
#endif
  servoUS_Write(90);
  servoCam_Write(90);
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
  if (us == 0) return 999;
  return (uint16_t)(us / 58);
}

static inline uint16_t clampDist(uint16_t d) {
  if (d == 0) return 999;
  if (d > 400) return 999;
  return d;
}

// =======================================================
//          AUTO: sweep continuo + decisione L/C/R
// =======================================================
static const uint16_t AUTO_OBS_CM   = 22;
static const uint16_t AUTO_GOOD_CM  = 35;

static const int16_t  AUTO_FWD_SPEED  = 160;
static const int16_t  AUTO_BACK_SPEED = 150;
static const int16_t  AUTO_TURN_SPEED = 180;

static const uint32_t AUTO_BACK_MS = 260;
static const uint32_t AUTO_TURN_MS = 380;
static const uint32_t AUTO_STOP_MS = 120;

enum AutoState : uint8_t { A_FWD=0, A_STOP, A_BACK, A_TURN };
static AutoState autoSt = A_FWD;
static uint32_t autoTs = 0;
static int8_t autoTurnDir = +1;

// sweep continuo servo US
static int usAngle = 90;
static int usDir = +1;
static const uint32_t US_SWEEP_MS = 55;

// misure “latched” per L/C/R
static uint16_t dL=999, dC=999, dR=999;
static uint32_t lastDistMs = 0;
static const uint32_t DIST_SAMPLE_MS = 70;

static inline void updateBucketsFromAngle(int ang, uint16_t cm) {
  cm = clampDist(cm);
  if (abs(ang - 90) <= 8) dC = cm;
  else if (ang <= (SERVO_US_MIN_ANG + 12)) dR = cm;
  else if (ang >= (SERVO_US_MAX_ANG - 12)) dL = cm;
}

static int8_t decideTurnDir() {
  uint16_t L = clampDist(dL), C = clampDist(dC), R = clampDist(dR);
  if (C >= AUTO_GOOD_CM) return 0;
  if (L > R + 3) return -1;
  if (R > L + 3) return +1;
  autoTurnDir = -autoTurnDir;
  return autoTurnDir;
}

static void autoReset() {
  autoSt = A_FWD;
  autoTs = millis();
  autoTurnDir = +1;
  dL = dC = dR = 999;
}

static void runAuto(bool backObs) {
  const uint32_t now = millis();

  static uint32_t tSweep = 0;
  if (now - tSweep >= US_SWEEP_MS) {
    tSweep = now;
    usAngle += usDir * 2;
    if (usAngle >= SERVO_US_MAX_ANG) { usAngle = SERVO_US_MAX_ANG; usDir = -1; }
    if (usAngle <= SERVO_US_MIN_ANG) { usAngle = SERVO_US_MIN_ANG; usDir = +1; }
    servoUS_Write(usAngle);
  }

  if (now - lastDistMs >= DIST_SAMPLE_MS) {
    lastDistMs = now;
    uint16_t cm = hcsr04ReadCm();
    updateBucketsFromAngle(usAngle, cm);
  }

  const bool frontObs = (clampDist(dC) < AUTO_OBS_CM);

  switch (autoSt) {
    case A_FWD: {
      motorsSetSmooth(AUTO_FWD_SPEED, AUTO_FWD_SPEED);
      if (frontObs) {
        autoSt = A_STOP;
        autoTs = now;
        motorsStop();
      }
    } break;

    case A_STOP: {
      motorsStop();
      if (now - autoTs >= AUTO_STOP_MS) {
        int8_t dir = decideTurnDir();
        if (dir == 0) {
          autoSt = A_FWD;
          autoTs = now;
        } else {
          uint16_t best = max(clampDist(dC), max(clampDist(dL), clampDist(dR)));
          if (best < (AUTO_OBS_CM + 6) && !backObs) {
            autoSt = A_BACK;
            autoTs = now;
          } else {
            autoTurnDir = dir;
            autoSt = A_TURN;
            autoTs = now;
          }
        }
      }
    } break;

    case A_BACK: {
      if (backObs) {
        motorsStop();
        autoSt = A_TURN;
        autoTs = now;
        autoTurnDir = -autoTurnDir;
        break;
      }

      motorsSetSmooth(-AUTO_BACK_SPEED, -AUTO_BACK_SPEED);
      if (now - autoTs >= AUTO_BACK_MS) {
        motorsStop();
        autoSt = A_TURN;
        autoTs = now;
        int8_t dir = decideTurnDir();
        if (dir != 0) autoTurnDir = dir;
      }
    } break;

    case A_TURN: {
      int16_t L = (autoTurnDir > 0) ? +AUTO_TURN_SPEED : -AUTO_TURN_SPEED;
      int16_t R = (autoTurnDir > 0) ? -AUTO_TURN_SPEED : +AUTO_TURN_SPEED;
      motorsSetSmooth(L, R);
      if (now - autoTs >= AUTO_TURN_MS) {
        motorsStop();
        autoSt = A_FWD;
        autoTs = now;
      }
    } break;
  }
}

// =======================================================
//                      Camera smoothing
// =======================================================
static int camCur = 90;
static int camTgt = 90;
static const uint32_t CAM_STEP_MS = 35;
static const int CAM_STEP_DEG = 1;
static uint32_t lastCamMs = 0;

// =======================================================
//                  STARTUP SELF-TEST
// =======================================================
static void selfTest() {
  Serial.println("\n--- SELF TEST ---");
  Serial.println("Servo US sweep...");
  for (int a = SERVO_US_MIN_ANG; a <= SERVO_US_MAX_ANG; a += 5) { servoUS_Write(a); delay(20); }
  for (int a = SERVO_US_MAX_ANG; a >= SERVO_US_MIN_ANG; a -= 5) { servoUS_Write(a); delay(20); }
  servoUS_Write(90);

  Serial.println("Motors: LEFT forward...");
  motorsSetSmooth(140, 0); delay(700);
  motorsStop(); delay(300);

  Serial.println("Motors: RIGHT forward...");
  motorsSetSmooth(0, 140); delay(700);
  motorsStop(); delay(300);

  Serial.println("Motors: BOTH forward...");
  motorsSetSmooth(140, 140); delay(700);
  motorsStop(); delay(300);

  Serial.println("--- END SELF TEST ---\n");
}

// =======================================================
//                         MAIN
// =======================================================
static EspNowReceiver rx;
static const uint32_t RX_TIMEOUT_MS = 250;

void setup() {
  Serial.begin(115200);
  delay(200);

  motorsInit();
  servosInit();
  hcsr04Init();
  irInit();

  Serial.print("RX STA MAC: ");
  Serial.println(WiFi.macAddress());

  if (!rx.begin()) {
    Serial.println("ESP-NOW init FAILED!");
    motorsStop();
    while (true) delay(1000);
  }

  motorsStop();
  autoReset();
  selfTest();
}

void loop() {
  ControlMsg msg{};
  uint32_t ageMs = 0;
  rx.getLatest(msg, ageMs);

  const uint32_t now = millis();
  const bool backObs = irBackObstacle();

  // FAILSAFE: solo in manuale stop se perdi radio
  if (!msg.autoMode && ageMs > RX_TIMEOUT_MS) {
    motorsStop();
    servoUS_Write(90);
    servoCam_Write(camCur);
    delay(5);
    return;
  }

  if (msg.autoMode) {
    runAuto(backObs);
    camTgt = 90;
  } else {
    autoReset();

    MotorCmd m = accelToMotorsManual(msg);

    // blocco retro se IR vede ostacolo
    const int ayEff = INVERT_THROTTLE_AXIS ? -msg.ay : msg.ay;
    const bool wantReverse = (ayEff < -DEADZONE);

    if (wantReverse && backObs) {
      if (m.left < 0)  m.left = 0;
      if (m.right < 0) m.right = 0;
    }

    motorsSetSmooth(m.left, m.right);

    // in manual US al centro
    servoUS_Write(90);

    // camera da az
    int az = clampInt((int)msg.az, -AX_AY_MAX, AX_AY_MAX);
    camTgt = mapInt(az, -AX_AY_MAX, AX_AY_MAX, 0, 180);
    camTgt = clampInt(camTgt, SERVO_CAM_MIN_ANG, SERVO_CAM_MAX_ANG);
  }

  // camera smoothing
  if (now - lastCamMs >= CAM_STEP_MS) {
    lastCamMs = now;
    if (camCur < camTgt) camCur += CAM_STEP_DEG;
    else if (camCur > camTgt) camCur -= CAM_STEP_DEG;
    servoCam_Write(camCur);
  }

  // debug
  static uint32_t tPrint = 0;
  if (now - tPrint >= 200) {
    tPrint = now;
    Serial.print("auto="); Serial.print((int)msg.autoMode);
    Serial.print(" ageMs="); Serial.print(ageMs);
    Serial.print(" ax="); Serial.print(msg.ax);
    Serial.print(" ay="); Serial.print(msg.ay);
    Serial.print(" L="); Serial.print(g_curL);
    Serial.print(" R="); Serial.print(g_curR);
    Serial.print(" IR_back="); Serial.print(backObs ? 1 : 0);
    Serial.print(" dL="); Serial.print(dL);
    Serial.print(" dC="); Serial.print(dC);
    Serial.print(" dR="); Serial.print(dR);
    Serial.print(" usAng="); Serial.print(usAngle);
    Serial.print(" st="); Serial.println((int)autoSt);
  }

  delay(5);
}
