#include <WiFi.h>

// dropar espNow per mandar indirizzo ip da server a client per permettergli di connettersi

// ===== Wi-Fi configuration (must match the AP) =====
const char* ssid     = "isma";
const char* password = "ismaele04";

// Car ESP32 AP IP (default for softAP is usually 192.168.4.1)
const char* host = "10.67.139.170";
const int   port = 80;

int targetSpeed  = 0;
int targetAngle  = 0;
int targetMode   = 0;  // 0=Manual

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Steering ESP32 - HTTP Client");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to AP");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("My IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // ---- Update fake commands (later: read from MSP432 via UART) ----
  static int step = 0;
  step++;

  targetSpeed = (step * 5) % 100;        // 0..95
  targetAngle = (step * 5) % 60 - 30;    // -30..29
  targetMode  = 0;                       // Manual

  // ---- Build the HTTP GET path with query string ----
  String url = "/cmd?speed=" + String(targetSpeed) +
               "&angle=" + String(targetAngle) +
               "&mode=" + String(targetMode);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(host);
  Serial.print(":");
  Serial.println(port);

  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("Connection failed");
    delay(1000);
    return;
  }

  Serial.print("Requesting URL: ");
  Serial.println(url);

  // ---- Send HTTP GET request ----
  client.print(String("GET ") + url + " HTTP/1.1\r\n");
  client.print(String("Host: ") + host + "\r\n");
  client.print("Connection: close\r\n");
  client.print("\r\n");

  // ---- Read response ----
  String responseHeaders;
  String responseBody;
  bool headersEnded = false;

  unsigned long timeout = millis() + 2000;
  while (client.connected() && millis() < timeout) {
    while (client.available()) {
      String line = client.readStringUntil('\n');

      if (!headersEnded) {
        // HTTP headers end with a blank line
        if (line == "\r" || line == "" || line == "\n") {
          headersEnded = true;
        } else {
          responseHeaders += line;
        }
      } else {
        responseBody += line;
      }
    }
  }

  client.stop();

  Serial.println("=== Response Headers ===");
  Serial.println(responseHeaders);
  Serial.println("=== Response Body ===");
  Serial.println(responseBody);

  // Here you could parse the JSON (e.g. speed/temp/battery),
  // using ArduinoJson or simple manual parsing if you want.

  delay(1000); // send command every second
}
