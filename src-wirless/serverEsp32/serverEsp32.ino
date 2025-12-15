#include <WiFi.h>

const char* apSsid     = "RedPanda_AP";
const char* apPassword = "12345678";

IPAddress apIP(192, 168, 4, 1);
IPAddress netmask(255, 255, 255, 0);

WiFiServer server(80);

int currentSpeed   = 0;
int currentTemp    = 25;
int currentBattery = 100;

int getIntParam(const String& url, const String& name, int def) {
  int idx = url.indexOf(name + "=");
  if (idx < 0) return def;
  int s = idx + name.length() + 1;
  int e = url.indexOf('&', s);
  if (e < 0) e = url.length();
  return url.substring(s, e).toInt();
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("\n[CAR] Starting OFFLINE AP + HTTP Server");

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, netmask);

  bool ok = WiFi.softAP(apSsid, apPassword);
  Serial.print("[CAR] softAP() result: ");
  Serial.println(ok ? "OK" : "FAIL");

  Serial.print("[CAR] AP SSID: ");
  Serial.println(apSsid);
  Serial.print("[CAR] AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
  Serial.println("[CAR] HTTP server started on port 80");
}

void loop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    Serial.print("[CAR] Stations connected: ");
    Serial.println(WiFi.softAPgetStationNum());
  }

  WiFiClient client = server.available();
  if (!client) return;

  Serial.println("\n[CAR] Client connected");

  String requestLine;
  bool firstLine = true;

  unsigned long deadline = millis() + 2000;
  while (client.connected() && millis() < deadline) {
    if (!client.available()) continue;

    String line = client.readStringUntil('\n');
    line.replace("\r", "");

    if (firstLine) {
      requestLine = line;
      firstLine = false;
    }

    if (line.length() == 0) break;
  }

  Serial.print("[CAR] ");
  Serial.println(requestLine);

  String method, url;
  int s1 = requestLine.indexOf(' ');
  int s2 = requestLine.indexOf(' ', s1 + 1);
  if (s1 > 0 && s2 > s1) {
    method = requestLine.substring(0, s1);
    url    = requestLine.substring(s1 + 1, s2);
  }

  if (method == "GET" && url.startsWith("/cmd")) {
    int speed = getIntParam(url, "speed", 0);
    int angle = getIntParam(url, "angle", 0);
    int mode  = getIntParam(url, "mode", 0);

    Serial.printf("[CAR] CMD speed=%d angle=%d mode=%d\n", speed, angle, mode);

    // Fake telemetry update
    currentSpeed   = speed;
    currentTemp    = 25 + abs(angle) / 10;
    currentBattery = max(0, currentBattery - 1);

    String json = "{";
    json += "\"status\":\"ok\",";
    json += "\"speed\":"   + String(currentSpeed) + ",";
    json += "\"temp\":"    + String(currentTemp) + ",";
    json += "\"battery\":" + String(currentBattery);
    json += "}";

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(json.length());
    client.println();
    client.print(json);
  } else if (method == "GET" && url == "/") {
    String msg = "RedPanda Car Server OK\nTry /cmd?speed=50&angle=0&mode=0\n";
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(msg.length());
    client.println();
    client.print(msg);
  } else {
    String msg = "Not found\n";
    client.println("HTTP/1.1 404 Not Found");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(msg.length());
    client.println();
    client.print(msg);
  }

  delay(5);
  client.stop();
  Serial.println("[CAR] Client disconnected");
}
