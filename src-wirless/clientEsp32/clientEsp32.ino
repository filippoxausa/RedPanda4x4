#include <WiFi.h>

const char* ssid     = "RedPanda_AP";
const char* password = "12345678";

IPAddress serverIP(192, 168, 4, 1);
const int serverPort = 80;

void connectToCarAP_strict() {
  Serial.print("[STEERING] Connecting to SSID: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(500);

  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  // Force DHCP (optional but helps)
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }

  Serial.println("\n[STEERING] WiFi connected!");
  Serial.print("[STEERING] Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("[STEERING] Gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("[STEERING] RSSI: ");
  Serial.println(WiFi.RSSI());
}

String httpGET(const String& path) {
  WiFiClient client;

  const int maxTries = 3;
  bool connected = false;
  for (int i = 0; i < maxTries; i++) {
    if (client.connect(serverIP, serverPort)) {
      connected = true;
      break;
    }
    delay(50);
  }

  if (!connected) return "";

  client.print("GET " + path + " HTTP/1.1\r\n");
  client.print("Host: 192.168.4.1\r\n");
  client.print("Connection: close\r\n\r\n");

  bool headersEnded = false;
  String body;

  unsigned long timeout = millis() + 2000;
  while ((client.connected() || client.available()) && millis() < timeout) {
    if (!client.available()) continue;

    String line = client.readStringUntil('\n');
    line.replace("\r", "");

    if (!headersEnded) {
      if (line.length() == 0) headersEnded = true;
    } else {
      body += line;
    }
  }

  client.stop();
  return body;
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("\n[STEERING] Starting OFFLINE HTTP client");
  connectToCarAP_strict();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[STEERING] WiFi dropped -> reconnecting...");
    connectToCarAP_strict();
  }

  static int t = 0;
  t++;

  int speed = (t * 5) % 100;
  int angle = (t * 3) % 60 - 30;
  int mode  = 0;

  String path = "/cmd?speed=" + String(speed) +
                "&angle=" + String(angle) +
                "&mode=" + String(mode);

  Serial.print("[STEERING] GET ");
  Serial.println(path);

  String body = httpGET(path);

  if (body.length() == 0) {
    Serial.println("[STEERING] Server not reachable (no body)");
    // If you want: force reconnect after N failures
  } else {
    Serial.print("[STEERING] Telemetry: ");
    Serial.println(body);
  }

  delay(500);
}
