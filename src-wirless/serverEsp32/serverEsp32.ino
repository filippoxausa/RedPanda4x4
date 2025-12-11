#include <WiFi.h>

// ===== Wi-Fi configuration =====
const char* ssid     = "isma";
const char* password = "ismaele04";  // must be at least 8 chars

// HTTP server on port 80
WiFiServer server(80);

// Fake telemetry state (later: replace with real data)
int currentSpeed   = 42;
int currentTemp    = 25;
int currentBattery = 87;

// Helper to extract integer parameter from URL query string
// Example: url = "/cmd?speed=50&angle=-10&mode=1"
// getIntParam(url, "speed", defaultVal) -> 50
int getIntParam(const String& url, const String& name, int defaultVal) {
  int idx = url.indexOf(name + "=");
  if (idx == -1) return defaultVal;

  int start = idx + name.length() + 1;
  int end = url.indexOf('&', start);
  if (end == -1) end = url.length();

  String valStr = url.substring(start, end);
  valStr.trim();
  if (valStr.length() == 0) return defaultVal;

  return valStr.toInt();
}

void waitForWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long startAttempt = millis();
  const unsigned long timeout = 15000;  // 15 seconds

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);

    if (millis() - startAttempt >= timeout) {
      Serial.println("\nFailed to connect. Restarting WiFi module...");
      WiFi.disconnect(true);
      delay(1000);
      WiFi.begin(ssid, password);
      startAttempt = millis();  // Reset timer
    }
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Starting ESP32 Car - Access Point + HTTP Server");

  waitForWiFi();

  server.begin();
  Serial.println("HTTP server started on port 80");
}

void loop() {
  WiFiClient client = server.available();
  if (!client) {
    // No client connected, nothing to do
    return;
  }

  Serial.println("Client connected");

  // ---- Read HTTP request ----
  String requestLine;
  String request;  // full request (for debugging, optional)
  bool firstLine = true;

  // Wait until client sends data
  unsigned long timeout = millis() + 2000; // 2s timeout
  while (client.connected() && millis() < timeout) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      line.trim();  // remove \r and spaces
      if (firstLine) {
        requestLine = line;
        firstLine = false;
      }
      request += line + "\n";

      // Empty line: end of headers
      if (line.length() == 0) {
        break;
      }
    }
  }

  Serial.println("=== HTTP Request Start ===");
  Serial.print(request);
  Serial.println("=== HTTP Request End ===");

  // ---- Parse request line: e.g. "GET /cmd?speed=50&angle=-10&mode=1 HTTP/1.1" ----
  String method;
  String url;
  String version;

  int firstSpace  = requestLine.indexOf(' ');
  int secondSpace = requestLine.indexOf(' ', firstSpace + 1);

  if (firstSpace != -1 && secondSpace != -1) {
    method  = requestLine.substring(0, firstSpace);
    url     = requestLine.substring(firstSpace + 1, secondSpace);
    version = requestLine.substring(secondSpace + 1);
  }

  Serial.print("Method: ");  Serial.println(method);
  Serial.print("URL: ");     Serial.println(url);
  Serial.print("Version: "); Serial.println(version);

  // Only handle GET /cmd...
  if (method == "GET" && url.startsWith("/cmd")) {
    // Extract parameters from query string
    int speedCmd  = getIntParam(url, "speed", 0);
    int angleCmd  = getIntParam(url, "angle", 0);
    int modeCmd   = getIntParam(url, "mode", 0);

    Serial.print("Parsed command -> speed=");
    Serial.print(speedCmd);
    Serial.print(" angle=");
    Serial.print(angleCmd);
    Serial.print(" mode=");
    Serial.println(modeCmd);

    // TODO: here you will apply these commands to your motor control logic

    // For now, update some fake telemetry state for fun
    currentSpeed   = speedCmd;               // pretend current speed follows command
    currentTemp    = 25 + (speedCmd / 10);   // fake effect
    currentBattery = max(0, currentBattery - 1);

    // ---- Build JSON response ----
    String json = "{";
    json += "\"status\":\"ok\",";
    json += "\"speed\":" + String(currentSpeed) + ",";
    json += "\"temp\":" + String(currentTemp) + ",";
    json += "\"battery\":" + String(currentBattery);
    json += "}";

    // ---- Send HTTP response ----
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(json.length());
    client.println();
    client.print(json);
  } else {
    // Unknown route -> 404
    String msg = "Not found\n";

    client.println("HTTP/1.1 404 Not Found");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(msg.length());
    client.println();
    client.print(msg);
  }

  // Give some time to flush
  delay(10);
  client.stop();
  Serial.println("Client disconnected");
}
