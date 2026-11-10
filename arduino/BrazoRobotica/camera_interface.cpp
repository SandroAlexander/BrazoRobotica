#include "camera_interface.h"
#include "config.h"
#include <Arduino.h>
#include "esp_http_server.h"

// AI-Thinker ESP32-CAM pin map
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

namespace {
bool           s_ready = false;
bool           s_psram = false;
volatile bool  s_streaming = false;
httpd_handle_t s_streamHttpd = nullptr;

const char* kBoundary     = "frame";
const char* kStreamType   = "multipart/x-mixed-replace;boundary=frame";
const char* kPartHeader   = "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t streamHandler(httpd_req_t* req) {
  if (!s_ready) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera not available");
    return ESP_FAIL;
  }
  esp_err_t res = httpd_resp_set_type(req, kStreamType);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  s_streaming = true;
  char part[96];
  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[CAM] Stream frame capture failed");
      res = ESP_FAIL;
      break;
    }
    const int hlen = snprintf(part, sizeof(part), kPartHeader, (unsigned)fb->len);
    res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;   // client disconnected
  }
  s_streaming = false;
  return res;
}
} // namespace

bool cameraBegin() {
  // Flash LED: force OFF before anything else
  pinMode(CAM_LED_PIN, OUTPUT);
  digitalWrite(CAM_LED_PIN, LOW);

  s_psram = psramFound();

  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (s_psram) {
    // Allocate for the largest size so the resolution can be raised later
    config.frame_size   = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size   = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  }

  const esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Init failed (0x%x) – arm control remains available\n", err);
    s_ready = false;
    return false;
  }
  s_ready = true;
  cameraApply(g_settings.camera);
  Serial.printf("[CAM] Ready (PSRAM: %s)\n", s_psram ? "yes" : "no");
  return true;
}

bool cameraIsReady()   { return s_ready; }
bool cameraHasPsram()  { return s_psram; }
bool cameraIsStreaming() { return s_streaming; }

void cameraSetLed(bool on) { digitalWrite(CAM_LED_PIN, on ? HIGH : LOW); }

bool cameraApply(const CameraSettings& in) {
  CameraSettings s = in;
  settingsSanitizeCamera(s);
  if (!s_psram && s.framesize > FRAMESIZE_SVGA) s.framesize = FRAMESIZE_SVGA;   // buffer limit
  cameraSetLed(s.led);
  if (!s_ready) return false;

  sensor_t* sensor = esp_camera_sensor_get();
  if (!sensor) return false;
  bool ok = true;
  ok &= sensor->set_framesize(sensor, (framesize_t)s.framesize) == 0;
  ok &= sensor->set_quality(sensor, s.quality) == 0;
  ok &= sensor->set_brightness(sensor, s.brightness) == 0;
  ok &= sensor->set_contrast(sensor, s.contrast) == 0;
  ok &= sensor->set_saturation(sensor, s.saturation) == 0;
  ok &= sensor->set_hmirror(sensor, s.hmirror ? 1 : 0) == 0;
  ok &= sensor->set_vflip(sensor, s.vflip ? 1 : 0) == 0;
  return ok;
}

bool cameraStartStreamServer() {
  if (s_streamHttpd) return true;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port      = STREAM_PORT;
  config.ctrl_port        = 32769;
  config.max_open_sockets = 3;
  config.lru_purge_enable = true;   // a new viewer replaces a stale one

  httpd_uri_t streamUri = {};
  streamUri.uri     = "/stream";
  streamUri.method  = HTTP_GET;
  streamUri.handler = streamHandler;

  if (httpd_start(&s_streamHttpd, &config) != ESP_OK) {
    Serial.println("[CAM] Failed to start stream server");
    s_streamHttpd = nullptr;
    return false;
  }
  httpd_register_uri_handler(s_streamHttpd, &streamUri);
  Serial.printf("[CAM] MJPEG stream on port %d /stream\n", STREAM_PORT);
  return true;
}

camera_fb_t* captureFrame() {
  if (!s_ready) return nullptr;
  camera_fb_t* fb = esp_camera_fb_get();
  // With GRAB_LATEST and 2 buffers the first frame may be stale: take a fresh one
  if (fb && s_psram && !s_streaming) {
    esp_camera_fb_return(fb);
    fb = esp_camera_fb_get();
  }
  return fb;
}

void releaseFrame(camera_fb_t* fb) {
  if (fb) esp_camera_fb_return(fb);
}
