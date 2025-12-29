#include <Arduino.h>
#include <Wire.h>
#include "motor_tb.h"

static const uint8_t MPU_ADDR = 0x68;     // prova 0x69 se non funziona [web:29]
static const uint8_t REG_WHO_AM_I = 0x75; // [web:29]
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B; // accel block starts here [web:50]

static uint8_t i2cRead8(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);            // repeated start pattern [web:43]
  Wire.requestFrom((int)addr, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

static void i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

static bool i2cReadBytes(uint8_t addr, uint8_t startReg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start [web:43]
  size_t n = Wire.requestFrom((int)addr, (int)len);
  if (n != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static int16_t toInt16(uint8_t hi, uint8_t lo) {
  return (int16_t)((hi << 8) | lo);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // I2C ESP32: SDA=21 SCL=22 (setup esplicito consigliato su ESP32)
  Wire.begin(21, 22);
  Wire.setClock(400000);

  motorsInit();

  // Wake-up MPU: PWR_MGMT_1 = 0
  i2cWrite8(MPU_ADDR, REG_PWR_MGMT_1, 0x00);

  uint8_t who = i2cRead8(MPU_ADDR, REG_WHO_AM_I);
  Serial.print("WHO_AM_I=0x");
  Serial.println(who, HEX);

  // MPU-6500 tipicamente 0x70 [web:29]
  if (who != 0x70) {
    Serial.println("ATTENZIONE: WHO_AM_I diverso da 0x70. Prova MPU_ADDR=0x69 o controlla cablaggi.");
  }
}

void loop() {
  uint8_t raw[6];
  if (!i2cReadBytes(MPU_ADDR, REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
    driveA(0);
    driveB(0);
    delay(50);
    return;
  }

  int16_t ax = toInt16(raw[0], raw[1]);
  int16_t ay = toInt16(raw[2], raw[3]);
  int16_t az = toInt16(raw[4], raw[5]);

  Serial.print("Axyz raw: ");
  Serial.print(ax); Serial.print('\t');
  Serial.print(ay); Serial.print('\t');
  Serial.println(az);

  // Logica richiesta su asse Y
  if (ay > 10000) {
    driveA(180);
    driveB(180);
  } else if (ay < -10000) {
    driveA(-180);
    driveB(-180);
  } else {
    driveA(0);
    driveB(0);
  }

  delay(50);
}
