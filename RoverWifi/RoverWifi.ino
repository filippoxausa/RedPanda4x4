#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>

#include "motor_tb.h"

// =====================
// Wi-Fi AP settings
// =====================
const char* ssid = "ROVER_AP";
const char* pass = "rover1234";   // min 8 chars

WebServer server(80);

// =====================
// Command state + failsafe
// =====================
volatile char lastCmd = 'S';
unsigned long lastCmdMs = 0;
const unsigned long FAILSAFE_MS = 400; // if no cmd for > 400ms -> STOP

// =====================
// Web handlers
// =====================
void handleRoot() {
  // Serve index.html from LittleFS
  File f = LittleFS.open("/index.html", "r");
  if (!f) {
    server.send(500, "text/plain", "index.html not found in LittleFS. Upload data/index.html to LittleFS.");
    return;
  }
  server.streamFile(f, "text/html; charset=utf-8");
  f.close();
}

void handleCmd() {
  if (!server.hasArg("d")) { server.send(400, "text/plain", "missing d"); return; }
  String d = server.arg("d");
  if (d.length() < 1) { server.send(400, "text/plain", "empty d"); return; }

  char cmd = d[0];
  if (cmd!='F' && cmd!='B' && cmd!='L' && cmd!='R' && cmd!='S') {
    server.send(400, "text/plain", "bad cmd");
    return;
  }

  lastCmd = cmd;
  lastCmdMs = millis();
  server.send(200, "text/plain", "ok");
}

// =====================
// Motor wrappers (A = left, B = right)
// =====================
static inline void stopAll() {
  driveA(0);
  driveB(0);
  // Se preferisci frenare:
  // brakeAll();
}

static inline void forward(int v = 180) {
  driveA(v);
  driveB(v);
}

static inline void backward(int v = 180) {
  driveA(-v);
  driveB(-v);
}

static inline void turnLeft(int v = 160) {
  // gira sul posto: sinistra indietro, destra avanti
  driveA(-v);
  driveB( v);
}

static inline void turnRight(int v = 160) {
  driveA( v);
  driveB(-v);
}

void setMotors(char cmd) {
  switch (cmd) {
    case 'F': forward(180);  break;
    case 'B': backward(180); break;
    case 'L': turnLeft(160); break;
    case 'R': turnRight(160);break;
    default:  stopAll();     break; // 'S' or timeout
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Init motors (TB6612)
  motorsInit();
  stopAll();

  // Init filesystem (LittleFS)
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  } else {
    Serial.println("LittleFS mounted");
  }

  // Wi-Fi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, pass);

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP()); // usually 192.168.4.1

  // Routes
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);

  // (opzionale) static files (se aggiungi css/js in futuro)
  server.serveStatic("/", LittleFS, "/");

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();

  // failsafe: if connection drops, stop rover
  if (millis() - lastCmdMs > FAILSAFE_MS) {
    lastCmd = 'S';
  }

  setMotors(lastCmd);
}
