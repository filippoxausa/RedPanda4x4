#include <Arduino.h>
#include <Wire.h>

static const uint8_t MPU_ADDR_0x68 = 0x68;   // AD0 low
static const uint8_t WHO_AM_I_REG  = 0x75;   // WHO_AM_I
static const uint8_t PWR_MGMT_1    = 0x6B;   // power management
static const uint8_t ACCEL_XOUT_H  = 0x3B;   // accel data start

uint8_t i2cRead8(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)addr, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

bool i2cReadBytes(uint8_t addr, uint8_t startReg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t n = Wire.requestFrom((int)addr, (int)len);
  if (n != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

int16_t toInt16(uint8_t hi, uint8_t lo) {
  return (int16_t)((hi << 8) | lo);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // ESP32: imposta esplicitamente SDA=21, SCL=22
  Wire.begin(21, 22);  // importante su ESP32 [web:31]
  Wire.setClock(400000);

  Serial.println("MPU6500 quick test (I2C)");

  // Wake-up: scrive 0 su PWR_MGMT_1
  i2cWrite8(MPU_ADDR_0x68, PWR_MGMT_1, 0x00);

  uint8_t who = i2cRead8(MPU_ADDR_0x68, WHO_AM_I_REG);
  Serial.print("WHO_AM_I = 0x");
  Serial.println(who, HEX);

  // Tipico MPU-6500: 0x70 [web:29]
  if (who != 0x70) {
    Serial.println("ATTENZIONE: WHO_AM_I non e' 0x70. Controlla indirizzo (0x68/0x69) e cablaggi.");
  }
}

void loop() {
  uint8_t raw[6];
  if (i2cReadBytes(MPU_ADDR_0x68, ACCEL_XOUT_H, raw, sizeof(raw))) {
    int16_t ax = toInt16(raw[0], raw[1]);
    int16_t ay = toInt16(raw[2], raw[3]);
    int16_t az = toInt16(raw[4], raw[5]);

    Serial.print("ACC raw: ");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.println(az);
  } else {
    Serial.println("Errore lettura accelerometro (I2C).");
  }

  delay(200);
}