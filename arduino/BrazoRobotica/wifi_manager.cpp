#include "wifi_manager.h"
#include "config.h"
#include "settings.h"
#include "emergency_stop.h"
#include "camera_interface.h"
#include "web_ui.h"

#include <WiFi.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <memory>

namespace {
const size_t kMaxMessageLen = 4096;
const int    kQueueLength   = 16;

AsyncWebServer s_server(HTTP_PORT);
AsyncWebSocket s_ws("/ws");
DNSServer      s_dns;
QueueHandle_t  s_queue = nullptr;
uint32_t       s_lastCleanupMs = 0;

// JPEG copy that outlives the camera frame buffer while the response is sent
struct FrameCopy {
  uint8_t* data = nullptr;
  size_t   len  = 0;
  FrameCopy(const uint8_t* src, size_t n) {
    data = (uint8_t*)(psramFound() ? ps_malloc(n) : malloc(n));
    if (data) { memcpy(data, src, n); len = n; }
  }
  ~FrameCopy() { free(data); }
};

void enqueue(uint32_t clientId, const char* text, size_t len) {
  WsMessage m;
  m.clientId = clientId;
  m.text = (char*)malloc(len + 1);
  if (!m.text) return;
  memcpy(m.text, text, len);
  m.text[len] = '\0';
  if (xQueueSend(s_queue, &m, 0) != pdTRUE) {
    free(m.text);
    s_ws.text(clientId, "{\"type\":\"error\",\"msg\":\"Controller busy, command dropped\"}");
  }
}

void handleWsText(AsyncWebSocketClient* client, const char* text, size_t len) {
  // Emergency stop bypasses the queue for minimum latency
  if (len < 256 && strstr(text, "emergency_stop")) {
    JsonDocument doc;
    if (!deserializeJson(doc, text, len) && doc["type"] == "emergency_stop") {
      estopTrigger();
    }
  }
  enqueue(client->id(), text, len);
}

void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type,
               void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("[WS] Client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      enqueue(client->id(), "{\"type\":\"get_config\"}", 21);
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("[WS] Client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      // Only complete, single-frame text messages are accepted
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        if (len > kMaxMessageLen) {
          client->text("{\"type\":\"error\",\"msg\":\"Message too large\"}");
          return;
        }
        handleWsText(client, (const char*)data, len);
      }
      break;
    }
    default:
      break;
  }
}

void setupRoutes() {
  s_server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    AsyncWebServerResponse* r = req->beginResponse_P(200, "text/html", (const uint8_t*)WEB_UI_HTML, strlen(WEB_UI_HTML));
    r->addHeader("Cache-Control", "no-cache");
    req->send(r);
  });

  // Single JPEG frame
  s_server.on("/capture", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!cameraIsReady()) { req->send(503, "text/plain", "Camera not available"); return; }
    camera_fb_t* fb = captureFrame();
    if (!fb) { req->send(500, "text/plain", "Capture failed"); return; }
    auto frame = std::make_shared<FrameCopy>(fb->buf, fb->len);
    releaseFrame(fb);
    if (!frame->data) { req->send(500, "text/plain", "Out of memory"); return; }

    AsyncWebServerResponse* r = req->beginResponse("image/jpeg", frame->len,
      [frame](uint8_t* out, size_t maxLen, size_t index) -> size_t {
        if (index >= frame->len) return 0;
        const size_t n = min(maxLen, frame->len - index);
        memcpy(out, frame->data + index, n);
        return n;
      });
    r->addHeader("Cache-Control", "no-store");
    r->addHeader("Content-Disposition", "inline; filename=brazo_capture.jpg");
    req->send(r);
  });

  s_server.onNotFound([](AsyncWebServerRequest* req) {
    // Unknown host/path (e.g. OS connectivity checks): send the user to the UI
    req->redirect(wifiUrl());
  });

  s_ws.onEvent(onWsEvent);
  s_server.addHandler(&s_ws);
}

String mdnsLabel() {
  String label = g_settings.hostname;
  label.replace('.', '-');
  return label;
}
} // namespace

bool wifiBegin() {
  s_queue = xQueueCreate(kQueueLength, sizeof(WsMessage));

  // Reset Wi-Fi radio to a clean state
  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(50);
  WiFi.mode(WIFI_AP);

  const IPAddress ip(192, 168, 4, 1), mask(255, 255, 255, 0);
  WiFi.softAPConfig(ip, ip, mask);

  Serial.printf("[WIFI] Starting AP \"%s\" on channel %d...\n", DEFAULT_AP_SSID, AP_CHANNEL);
  if (!WiFi.softAP(DEFAULT_AP_SSID, DEFAULT_AP_PASSWORD, AP_CHANNEL, 0, AP_MAX_CLIENTS)) {
    Serial.println("[WIFI] ERROR: SoftAP start failed!");
    return false;
  }

  // Set maximum transmission power and low latency mode after AP is initialized
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(false);
  WiFi.softAPsetHostname(mdnsLabel().c_str());

  // DNS: Wildcard "*" resolves all captive portal probes and custom hostnames to the AP IP
  s_dns.setErrorReplyCode(DNSReplyCode::NoError);
  s_dns.setTTL(60);
  s_dns.start(53, "*", ip);

  if (MDNS.begin(mdnsLabel().c_str())) MDNS.addService("http", "tcp", HTTP_PORT);

  setupRoutes();
  s_server.begin();

  Serial.printf("[WIFI] AP \"%s\" online!\n", DEFAULT_AP_SSID);
  Serial.printf("[WIFI] Connect to: http://%s/ (or http://%s.local/ or %s)\n",
                ip.toString().c_str(), mdnsLabel().c_str(), wifiUrl().c_str());
  return true;
}

void wifiLoop() {
  s_dns.processNextRequest();
  const uint32_t now = millis();
  if (now - s_lastCleanupMs > 1000) {
    s_lastCleanupMs = now;
    s_ws.cleanupClients();
  }
}

bool wifiNextMessage(WsMessage& msg) {
  return s_queue && xQueueReceive(s_queue, &msg, 0) == pdTRUE;
}

void wsSend(uint32_t clientId, const char* text) { s_ws.text(clientId, text); }

void wsBroadcast(const char* text) {
  if (s_ws.count() > 0) s_ws.textAll(text);
}

size_t wsClientCount() { return s_ws.count(); }

String wifiUrl() { return String("http://") + g_settings.hostname + "/"; }
