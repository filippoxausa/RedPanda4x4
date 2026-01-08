#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

// ====== MAC del RX (MOTORI) ======
static const uint8_t RX_MAC[6] = { 0xE0,0x8C,0xFE,0x2E,0x96,0x7C }; // <-- METTI IL TUO


// ====== MPU6500 I2C ======
static const uint8_t MPU_ADDR = 0x68;            // se AD0 alto -> 0x69
static const uint8_t REG_WHO_AM_I = 0x75;        // WHO_AM_I
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;    // blocco accel (6 byte) [web:1]

typedef struct __attribute__((packed)) {
  int16_t speed; // -255..+255
} ControlMsg;

static uint8_t i2cRead8(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
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
  if (Wire.endTransmission(false) != 0) return false;
  size_t n = Wire.requestFrom((int)addr, (int)len);
  if (n != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static int16_t toInt16(uint8_t hi, uint8_t lo) {
  return (int16_t)((hi << 8) | lo);
}

static void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  (void)mac;
  Serial.print("ESP-NOW send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // I2C ESP32: SDA=21 SCL=22
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // Wake MPU
  i2cWrite8(MPU_ADDR, REG_PWR_MGMT_1, 0x00);

  uint8_t who = i2cRead8(MPU_ADDR, REG_WHO_AM_I);
  Serial.print("WHO_AM_I=0x");
  Serial.println(who, HEX);

  // ESP-NOW init
  WiFi.mode(WIFI_STA);
  Serial.print("TX STA MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, RX_MAC, 6);  // peer MAC [web:69]
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("esp_now_add_peer failed");
    while (true) delay(1000);
  }
}

void loop() {
  ControlMsg msg{0};

  uint8_t raw[6];
  if (i2cReadBytes(MPU_ADDR, REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
    int16_t ay = toInt16(raw[2], raw[3]); // AY nel blocco accel [web:1]

    if (ay > 10000) msg.speed = 255;
    else if (ay < -10000) msg.speed = -255;
    else msg.speed = 0;

    Serial.print("ay=");
    Serial.print(ay);
    Serial.print(" -> speed=");
    Serial.println(msg.speed);
  } else {
    msg.speed = 0; // fail-safe
    Serial.println("MPU read fail -> speed=0");
  }

  esp_now_send(RX_MAC, (uint8_t*)&msg, sizeof(msg));
  delay(50);
}
