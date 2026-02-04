#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include "control_msg.h"
#include "espnow_receiver.h"
#include "control_logic.h"
#include "motor_tb6612.h"

static const char* WIFI_SSID = "METTI_SSID";
static const char* WIFI_PASS = "METTI_PASSWORD";

static WebServer server(80);

static EspNowReceiver nowRx;

static volatile int16_t  s_webAx = 0;
static volatile int16_t  s_webAy = 0;
static volatile int16_t  s_webAz = 0;
static volatile uint32_t s_webLastMs = 0;

static const uint32_t WEB_ACTIVE_MS = 350;

static const uint32_t FAILSAFE_MS = 600;

static const int16_t WEB_RANGE = 16000;

static inline int16_t clamp16(int v, int lo, int hi) {
  if (v < lo) return (int16_t)lo;
  if (v > hi) return (int16_t)hi;
  return (int16_t)v;
}

static void wifiConnectSTA() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi NOT connected (web will still start if possible).");
  }
}

static void handleRoot() {
  File f = LittleFS.open("/index.html", "r");
  if (!f) {
    server.send(500, "text/plain",
                "index.html not found in LittleFS. Put it in data/index.html and upload LittleFS.");
    return;
  }
  server.streamFile(f, "text/html; charset=utf-8");
  f.close();
}

static void handleCmd() {
  if (!server.hasArg("x") || !server.hasArg("y")) {
    server.send(400, "text/plain", "missing x or y");
    return;
  }

  int x = server.arg("x").toInt();
  int y = server.arg("y").toInt();
  int z = server.hasArg("z") ? server.arg("z").toInt() : 0;

  s_webAx = clamp16(x, -WEB_RANGE, WEB_RANGE);
  s_webAy = clamp16(y, -WEB_RANGE, WEB_RANGE);
  s_webAz = clamp16(z, -WEB_RANGE, WEB_RANGE);
  s_webLastMs = millis();

  server.send(200, "text/plain", "ok");
}

static void handleStatus() {
  ControlMsg a{};
  uint32_t espAge = 0;
  nowRx.getLatest(a, espAge);

  uint32_t now = millis();
  uint32_t webAge = now - (uint32_t)s_webLastMs;

  String json = "{";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"web\":{\"ax\":" + String((int)s_webAx) + ",\"ay\":" + String((int)s_webAy) + ",\"az\":" + String((int)s_webAz) + ",\"age_ms\":" + String((unsigned long)webAge) + "},";
  json += "\"espnow\":{\"ax\":" + String((int)a.ax) + ",\"ay\":" + String((int)a.ay) + ",\"az\":" + String((int)a.az) + ",\"age_ms\":" + String((unsigned long)espAge) + "}";
  json += "}";

  server.send(200, "application/json", json);
}

static void getBestControl(ControlMsg &out, uint32_t &ageMs) {
  uint32_t now = millis();
  uint32_t webAge = now - (uint32_t)s_webLastMs;

  if (webAge <= WEB_ACTIVE_MS) {
    out.ax = s_webAx;
    out.ay = s_webAy;
    out.az = s_webAz;
    ageMs = webAge;
    return;
  }

  nowRx.getLatest(out, ageMs);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  motorsInit();
  motorsStop();

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  } else {
    Serial.println("LittleFS mounted");
  }

  wifiConnectSTA();

  Serial.print("RX STA MAC: ");
  Serial.println(WiFi.macAddress());

  if (!nowRx.begin()) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/status", handleStatus);   
  server.serveStatic("/", LittleFS, "/");
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();

  ControlMsg a{};
  uint32_t ageMs = 0;
  getBestControl(a, ageMs);

  if (ageMs > FAILSAFE_MS) {
    motorsStop();
  } else {
    MotorCmd cmd = accelToMotors(a);
    motorsSet(cmd.left, cmd.right);
  }

  delay(5);
}
