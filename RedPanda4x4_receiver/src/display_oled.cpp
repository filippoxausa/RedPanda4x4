#include "display_oled.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =======================================================
//                    Config
// =======================================================
static const int SCREEN_W  = 128;
static const int SCREEN_H  = 64;
static const int OLED_ADDR = 0x3C;

static Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);

// =======================================================
//                   Public API
// =======================================================

bool displayInit() {
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] init FAILED (addr 0x3C?). Try 0x3D if it doesn't work.");
    return false;
  }
  oled.clearDisplay();
  oled.display();
  return true;
}

void drawHappy() {
  oled.clearDisplay();
  // eyes
  oled.fillCircle(44, 24, 5, SSD1306_WHITE);
  oled.fillCircle(84, 24, 5, SSD1306_WHITE);
  // smile
  oled.drawCircle(64, 44, 20, SSD1306_WHITE);
  oled.fillRect(44, 24, 40, 20, SSD1306_BLACK);
  oled.display();
}

void drawSad() {
  oled.clearDisplay();
  // eyes
  oled.fillCircle(44, 24, 5, SSD1306_WHITE);
  oled.fillCircle(84, 24, 5, SSD1306_WHITE);
  // sad mouth
  oled.drawCircle(64, 60, 20, SSD1306_WHITE);
  oled.fillRect(44, 60, 40, 20, SSD1306_BLACK);
  oled.display();
}

void drawProjectName() {
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 22);
  oled.print("RedPanda");
  oled.setCursor(0, 44);
  oled.print("4x4");
  oled.display();
}
