#include <WebServer.h>
#include <WiFi.h>
#include <esp32cam.h>

//THIS PROGRAM SENDS IMAGE IF IT IS PLACED IN WEB IP, BUT IF IT IS PLACED IN PYTHON IT SENDS VIDEO THROUGH THE ITERATIONS. . . (IF IT WORKS IN PYTHON)
const char* WIFI_SSID = "ESP cam";
const char* WIFI_PASS = "12345678";

WebServer server(80); //server on port 80

static auto loRes = esp32cam::Resolution::find(320, 240); //low resolution
static auto hiRes = esp32cam::Resolution::find(800, 600); //high resolution
//static auto hiRes = esp32cam::Resolution::find(640, 480); //high resolution (for fps rates) (IP CAM APP)

void
serveJpg() //capture image .jpg
{
  auto frame = esp32cam::capture();
  if (frame == nullptr) {
    Serial.println("Capture Fail");
    server.send(503, "", "");
    return;
  }
  Serial.printf("CAPTURE OK %dx%d %db\n", frame->getWidth(), frame->getHeight(),
                static_cast<int>(frame->size()));

  server.setContentLength(frame->size());
  server.send(200, "image/jpeg");
  WiFiClient client = server.client();
  frame->writeTo(client);  //and send to a client (in this case it will be python)
}

void
handleJpgLo()  //allows to send low resolution image
{
  if (!esp32cam::Camera.changeResolution(loRes)) {
    Serial.println("SET-LO-RES FAIL");
  }
  serveJpg();
}

void
handleJpgHi() //allows to send high resolution image
{
  if (!esp32cam::Camera.changeResolution(hiRes)) {
    Serial.println("SET-HI-RES FAIL");
  }
  serveJpg();
}

void setup()
{
  Serial.begin(115200);
  Serial.println();

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

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS); //connect to the WiFi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/cam-lo.jpg");//to connect IP low res

  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/cam-hi.jpg");//to connect high res IP
  server.on("/cam-lo.jpg",handleJpgLo);//send to the server
  server.on("/cam-hi.jpg", handleJpgHi);

  server.begin();
}

void loop()
{
  server.handleClient();
}