#include <WebServer.h>
#include <WiFi.h>
#include <esp32cam.h>

const char* WIFI_SSID = "redpanda";
const char* WIFI_PASS = "aaaaaaaa";

WebServer server(80);

// Risoluzioni
static auto loRes = esp32cam::Resolution::find(320, 240);
static auto hiRes = esp32cam::Resolution::find(800, 600);

// ====== JPEG singola (come prima) ======
static void serveJpg() {
  auto frame = esp32cam::capture();
  if (frame == nullptr) {
    Serial.println("Capture Fail");
    server.send(503, "text/plain", "capture fail");
    return;
  }

  Serial.printf("CAPTURE OK %dx%d %db\n",
                frame->getWidth(),
                frame->getHeight(),
                (int)frame->size());

  server.setContentLength(frame->size());
  server.send(200, "image/jpeg");
  WiFiClient client = server.client();
  frame->writeTo(client);
}

static void handleJpgLo() {
  if (!esp32cam::Camera.changeResolution(loRes)) {
    Serial.println("SET-LO-RES FAIL");
  }
  serveJpg();
}

static void handleJpgHi() {
  if (!esp32cam::Camera.changeResolution(hiRes)) {
    Serial.println("SET-HI-RES FAIL");
  }
  serveJpg();
}

// ====== STREAM MJPEG ======
static void streamMjpeg(const esp32cam::Resolution& res) {
  // Prova a settare risoluzione
  if (!esp32cam::Camera.changeResolution(res)) {
    Serial.println("SET-STREAM-RES FAIL");
  }

  WiFiClient client = server.client();

  // Header MJPEG
  client.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
    "Cache-Control: no-cache\r\n"
    "Connection: close\r\n"
    "\r\n"
  );

  Serial.println("MJPEG stream started");

  // Loop di streaming finché il client resta connesso
  while (client.connected()) {
    auto frame = esp32cam::capture();
    if (frame == nullptr) {
      Serial.println("Capture Fail (stream)");
      delay(30);
      continue;
    }

    // Parte "frame"
    client.print("--frame\r\n");
    client.print("Content-Type: image/jpeg\r\n");
    client.print("Content-Length: ");
    client.print(frame->size());
    client.print("\r\n\r\n");

    // Scrive il JPEG
    frame->writeTo(client);
    client.print("\r\n");

    // Se il client ha chiuso, esci
    if (!client.connected()) break;

    // Regola FPS (aumenta per più fluido, diminuisci se lagga)
    delay(50); // ~20 fps teorici, in pratica dipende dalla res
    yield();
  }

  Serial.println("MJPEG stream ended");
}

static void handleStreamLo() { streamMjpeg(loRes); }
static void handleStreamHi() { streamMjpeg(hiRes); }
static void handleStream()   { streamMjpeg(hiRes); }

void setup() {
  Serial.begin(115200);
  Serial.println();

  // Camera init
  {
    using namespace esp32cam;
    Config cfg;
    cfg.setPins(pins::AiThinker);
    cfg.setResolution(hiRes);
    cfg.setBufferCount(2);
    cfg.setJpeg(80);
    bool ok = Camera.begin(cfg);
    Serial.println(ok ? "CAMERA OK" : "CAMERA FAIL");
  }

  // WiFi
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("IP: http://");
  Serial.println(WiFi.localIP());

  // Endpoints foto
  server.on("/cam-lo.jpg", handleJpgLo);
  server.on("/cam-hi.jpg", handleJpgHi);

  // Endpoints stream
  server.on("/stream-lo", HTTP_GET, handleStreamLo);
  server.on("/stream-hi", HTTP_GET, handleStreamHi);
  server.on("/stream",    HTTP_GET, handleStream);

  Serial.println("Single JPG:");
  Serial.printf("  http://%s/cam-lo.jpg\n", WiFi.localIP().toString().c_str());
  Serial.printf("  http://%s/cam-hi.jpg\n", WiFi.localIP().toString().c_str());

  Serial.println("MJPEG Stream:");
  Serial.printf("  http://%s/stream-lo\n", WiFi.localIP().toString().c_str());
  Serial.printf("  http://%s/stream-hi\n", WiFi.localIP().toString().c_str());
  Serial.printf("  http://%s/stream\n",    WiFi.localIP().toString().c_str());

  server.begin();
}

void loop() {
  server.handleClient();
}
