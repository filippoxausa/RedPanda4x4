#include <Arduino.h>
#include <esp_system.h>

void setup() {
  Serial.begin(115200);
  delay(200);

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA); // reads STA MAC

  Serial.printf("STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void loop() {}