#include <Arduino.h>

static const int PIN_AIN1 = 14;
static const int PIN_AIN2 = 27;
static const int PIN_PWMA = 26;  // deve essere un GPIO con PWM (su ESP32 quasi tutti vanno bene)
static const int PIN_STBY = 25;


static const int PWM_CH   = 0;
static const int PWM_FREQ = 20000; // 20 kHz
static const int PWM_RES  = 8;     // 8 bit => duty 0..255

void motorEnable(bool en) {
  digitalWrite(PIN_STBY, en ? HIGH : LOW); // STBY HIGH abilita il driver
}

void motorDrive(int speed) {
  // speed: -255..+255
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    ledcWrite(PWM_CH, speed);
  } else if (speed < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    ledcWrite(PWM_CH, -speed);
  } else {
    // stop "coast": IN1=IN2=LOW
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    ledcWrite(PWM_CH, 0);
  }
}

void motorBrake() {
  // freno: IN1=IN2=HIGH (short brake)
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, HIGH);
  ledcWrite(PWM_CH, 0);
}

void setup() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  // PWM setup: ledcSetup + ledcAttachPin + ledcWrite [web:94]
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWMA, PWM_CH);
  ledcWrite(PWM_CH, 0);

  motorEnable(true);
  motorDrive(0);
}

void loop() {
  motorDrive(180);   // avanti ~70%
  delay(1500);

  motorDrive(0);     // stop
  delay(400);

  motorDrive(-180);  // indietro
  delay(1500);

  motorBrake();      // freno
  delay(600);
}
