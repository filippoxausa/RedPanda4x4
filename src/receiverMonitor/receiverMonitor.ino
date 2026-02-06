#include <Arduino.h>
#include <TFT_eSPI.h>

TFT_eSPI tft;
HardwareSerial Link(2);

static const int UART_RX = 27;
static const int UART_TX = -1;

// Dati (nuovo protocollo)
int modeNow = 1;
int btn = 0;
float yawDps = 0.0f;
float headingDeg = 0.0f;
String dirStr = "STRAIGHT";

// Throttle redraw
uint32_t lastDrawMs = 0;
const uint32_t DRAW_PERIOD_MS = 100; // 10 Hz
volatile bool dirty = false;

bool parseDataLine(const String& s) {
  // atteso: "D,mode,btn,yaw,head,dir"
  if (!s.startsWith("D,")) return false;

  int p0 = 2; // dopo "D,"
  int c1 = s.indexOf(',', p0);
  int c2 = s.indexOf(',', c1 + 1);
  int c3 = s.indexOf(',', c2 + 1);
  int c4 = s.indexOf(',', c3 + 1);
  if (c1 < 0 || c2 < 0 || c3 < 0 || c4 < 0) return false;

  modeNow = s.substring(p0, c1).toInt();
  btn = s.substring(c1 + 1, c2).toInt();
  yawDps = s.substring(c2 + 1, c3).toFloat();
  headingDeg = s.substring(c3 + 1, c4).toFloat();
  dirStr = s.substring(c4 + 1);
  dirStr.trim();
  return true;
}

const char* modeText(int m) {
  return (m == 2) ? "MODE: GYRO" : "MODE: JOYSTICK";
}

void drawUI() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);

  tft.setCursor(10, 10);
  tft.println("RedPanda4x4");

  tft.setCursor(10, 50);
  tft.println(modeText(modeNow));

  tft.setCursor(10, 80);
  tft.print("Btn: ");
  tft.println(btn ? "PRESSED" : "released");

  tft.setCursor(10, 120);
  tft.printf("Yaw: %.1f dps\n", yawDps);

  tft.setCursor(10, 150);
  tft.printf("Head: %.1f deg\n", headingDeg);

  tft.setCursor(10, 190);
  tft.print("Dir: ");
  tft.println(dirStr);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Link.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);

  tft.setCursor(10, 10);
  tft.println("Waiting UART2 RX=27");
  tft.setCursor(10, 40);
  tft.println("Expect: D,mode,btn,yaw,head,dir");

  Serial.println("[DISPLAY] Ready. Reading ONLY UART2 (Link).");
}

void loop() {
  static String buf;

  while (Link.available()) {
    char ch = (char)Link.read();
    if (ch == '\r') continue;

    if (ch == '\n') {
      buf.trim();
      if (buf.length()) {
        Serial.print("[DISPLAY] RX: ");
        Serial.println(buf);

        if (parseDataLine(buf)) {
          dirty = true;
        } else {
          Serial.println("[DISPLAY] ignored (not D,...)");
        }
      }
      buf = "";
    } else {
      if (buf.length() < 160) buf += ch;
      else buf = "";
    }
  }

  uint32_t now = millis();
  if (dirty && (now - lastDrawMs) >= DRAW_PERIOD_MS) {
    lastDrawMs = now;
    dirty = false;
    drawUI();
  }
}