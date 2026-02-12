#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// =========================
//   CAMERA MODEL SELECT
// =========================
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// =========================
//        WIFI
// =========================
const char* ssid     = "TIM-75629655";
const char* password = "3eSzYthZGfy2tURhSxDhDhe9";

// =========================
//    ROTATION OPTIONS
// =========================
// true = ruota di 180° (flip+mirror) -> utile se è sottosopra
// false = nessuna rotazione
static const bool ROTATE_180 = false;

// =========================
//   STREAM SETTINGS
// =========================
// FRAMESIZE_VGA  (640x480)  -> “decente” e abbastanza fluido
// FRAMESIZE_SVGA (800x600)  -> più dettaglio, più pesante
static framesize_t STREAM_SIZE = FRAMESIZE_VGA;

// qualità JPEG: più basso = più qualità e più peso
// 10-12 buono; 12 più leggero
static int JPEG_QUALITY = 12;

static httpd_handle_t server = NULL;

static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char* STREAM_BOUNDARY     = "\r\n--frame\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t root_handler(httpd_req_t *req) {
  const char html[] =
    "<!doctype html><html><head>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ESP32-CAM Stream</title></head><body style='margin:0;background:#111;'>"
    "<div style='display:flex;justify-content:center;align-items:center;height:100vh;'>"
    "<img src='/stream' style='max-width:100%;max-height:100%;' />"
    "</div></body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, strlen(html));
}

static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;

  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, proxy-revalidate");
  httpd_resp_set_hdr(req, "Pragma", "no-cache");

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      // Se fallisce, evita crash e riprova
      delay(10);
      continue;
    }

    // invia boundary
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

    // header del frame
    char part_buf[64];
    int hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, (unsigned)fb->len);
    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

    // bytes jpeg
    res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;

    // piccolo respiro per stabilità
    // (puoi mettere 0 per massimo FPS, ma a volte aiuta)
    delay(1);
  }

  // chiude chunked
  httpd_resp_send_chunk(req, NULL, 0);
  return res;
}

static void start_webserver() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 8;

  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_uri_t uri_root = {
      .uri       = "/",
      .method    = HTTP_GET,
      .handler   = root_handler,
      .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &uri_root);

    httpd_uri_t uri_stream = {
      .uri       = "/stream",
      .method    = HTTP_GET,
      .handler   = stream_handler,
      .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &uri_stream);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  delay(200);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // frame buffers
  if (psramFound()) {
    config.frame_size   = STREAM_SIZE;
    config.jpeg_quality = JPEG_QUALITY;
    config.fb_count     = 2;
    config.grab_mode    = CAMERA_GRAB_LATEST;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    // senza PSRAM: vai più conservativo
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 14;
    config.fb_count     = 1;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location  = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while(true) delay(1000);
  }

  sensor_t *s = esp_camera_sensor_get();

  // ---- Rotazione "semplice" (solo 180°) ----
  if (ROTATE_180) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  } else {
    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);
  }

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  Serial.print("Open: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/  (page)");
  Serial.print("Stream: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/stream");

  start_webserver();
}

void loop() {
  delay(1000);
}
