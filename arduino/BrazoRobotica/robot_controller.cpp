#include "robot_controller.h"
#include "config.h"
#include "settings.h"
#include "pwm_driver.h"
#include "ik_solver.h"
#include "emergency_stop.h"
#include "camera_interface.h"
#include "wifi_manager.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ctype.h>

namespace {
uint32_t s_lastTelemetryMs = 0;
uint32_t s_restartAtMs = 0;
bool     s_wasEstop = false;

// ------------------------------------------------------------------ helpers
void sendJson(uint32_t cid, JsonDocument& doc) {
  String out;
  serializeJson(doc, out);
  if (cid) wsSend(cid, out.c_str()); else wsBroadcast(out.c_str());
}

void sendMessage(uint32_t cid, const char* type, const char* msg) {
  JsonDocument doc;
  doc["type"] = type;
  doc["msg"] = msg;
  sendJson(cid, doc);
}

void sendLocked(uint32_t cid) {
  JsonDocument doc;
  doc["type"] = "error";
  doc["msg"] = "Emergency stop active – movement locked";
  doc["remaining_ms"] = estopRemainingMs();
  sendJson(cid, doc);
}

// One decimal is plenty for the UI and keeps frames small
inline auto f1(float v) { return serialized(String(v, 1)); }

void buildConfig(JsonDocument& doc) {
  doc["type"] = "config";
  doc["hostname"] = g_settings.hostname;
  doc["url"] = wifiUrl();
  doc["ssid"] = DEFAULT_AP_SSID;
  doc["streamPort"] = STREAM_PORT;
  doc["lockoutMs"] = EMERGENCY_LOCKOUT_MS;
  doc["maxSpeed"] = f1(g_settings.maxSpeedDps);
  doc["psram"] = cameraHasPsram();

  JsonArray joints = doc["joints"].to<JsonArray>();
  for (int i = 0; i < NUM_JOINTS; ++i) {
    const JointCalibration& c = g_settings.joints[i];
    JsonObject j = joints.add<JsonObject>();
    j["name"] = JOINT_NAMES[i];
    j["channel"] = c.channel;
    j["minUs"] = c.minPulseUs;
    j["maxUs"] = c.maxPulseUs;
    j["min"] = f1(c.minAngle);
    j["max"] = f1(c.maxAngle);
    j["home"] = f1(c.homeAngle);
    j["kinZero"] = f1(c.kinZero);
    j["kinDir"] = c.kinDir;
  }

  JsonObject m = doc["model"].to<JsonObject>();
  m["L0"] = f1(g_settings.model.L0);
  m["L1"] = f1(g_settings.model.L1);
  m["L2"] = f1(g_settings.model.L2);
  m["L3"] = f1(g_settings.model.L3);

  const CameraSettings& cs = g_settings.camera;
  JsonObject cam = doc["camera"].to<JsonObject>();
  cam["ready"] = cameraIsReady();
  cam["framesize"] = cs.framesize;
  cam["quality"] = cs.quality;
  cam["brightness"] = cs.brightness;
  cam["contrast"] = cs.contrast;
  cam["saturation"] = cs.saturation;
  cam["hmirror"] = cs.hmirror;
  cam["vflip"] = cs.vflip;
  cam["led"] = cs.led;
}

void sendConfig(uint32_t cid) {
  JsonDocument doc;
  buildConfig(doc);
  sendJson(cid, doc);
}

void broadcastStatus() {
  float cur[NUM_JOINTS], tgt[NUM_JOINTS], tip[3];
  getJointAngles(cur);
  getTargetAngles(tgt);
  ikForward(cur, g_settings.joints, g_settings.model, tip, nullptr);
  const uint32_t remaining = estopRemainingMs();

  char buf[512];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"status\",\"angles\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
    "\"targets\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],\"tip\":[%.1f,%.1f,%.1f],"
    "\"moving\":%s,\"estop\":%s,\"estop_ms\":%u,\"pwm\":%s,\"camera\":%s,\"streaming\":%s,"
    "\"uptime_ms\":%lu,\"clients\":%u,\"heap\":%u}",
    cur[0], cur[1], cur[2], cur[3], cur[4], cur[5],
    tgt[0], tgt[1], tgt[2], tgt[3], tgt[4], tgt[5],
    tip[0], tip[1], tip[2],
    isMoving() ? "true" : "false", remaining ? "true" : "false", (unsigned)remaining,
    pwmIsOnline() ? "true" : "false", cameraIsReady() ? "true" : "false",
    cameraIsStreaming() ? "true" : "false",
    (unsigned long)millis(), (unsigned)wsClientCount(), (unsigned)ESP.getFreeHeap());
  wsBroadcast(buf);
}

bool isMovementCommand(const char* type, JsonDocument& doc) {
  if (!strcmp(type, "set_joint") || !strcmp(type, "set_all") || !strcmp(type, "home")) return true;
  if (!strcmp(type, "ik_target")) return doc["move"] | true;
  return false;
}

// ------------------------------------------------------------------ handlers
void handleIK(uint32_t cid, JsonDocument& doc) {
  if (!doc["x"].is<float>() || !doc["y"].is<float>() || !doc["z"].is<float>()) {
    sendMessage(cid, "error", "ik_target requires numeric x, y, z");
    return;
  }
  IKTarget t;
  t.x = doc["x"]; t.y = doc["y"]; t.z = doc["z"];
  t.useOrientation = doc["orient"] | false;
  t.pitchDeg = doc["pitch"] | -90.0f;
  const bool move = doc["move"] | true;

  float seed[NUM_JOINTS];
  getTargetAngles(seed);   // continue from where the arm is heading

  const uint32_t t0 = micros();
  const IKResult r = ikSolve(t, seed, g_settings.joints, g_settings.model);
  const uint32_t dt = micros() - t0;

  const bool moved = r.success && move && !estopIsActive();
  if (moved) setAllJointAngles(r.angles);

  JsonDocument out;
  out["type"] = "ik_result";
  out["success"] = r.success;
  out["moved"] = moved;
  out["iterations"] = r.iterations;
  out["posErr"] = f1(r.positionError);
  out["oriErr"] = serialized(String(r.orientationError, 3));
  out["time_us"] = dt;
  JsonArray a = out["angles"].to<JsonArray>();
  for (int i = 0; i < NUM_JOINTS; ++i) a.add(f1(r.angles[i]));
  sendJson(cid, out);
}

void handleCamera(uint32_t cid, JsonDocument& doc) {
  CameraSettings c = g_settings.camera;
  if (doc["framesize"].is<int>())  c.framesize  = doc["framesize"];
  if (doc["quality"].is<int>())    c.quality    = doc["quality"];
  if (doc["brightness"].is<int>()) c.brightness = doc["brightness"];
  if (doc["contrast"].is<int>())   c.contrast   = doc["contrast"];
  if (doc["saturation"].is<int>()) c.saturation = doc["saturation"];
  if (doc["hmirror"].is<bool>())   c.hmirror    = doc["hmirror"];
  if (doc["vflip"].is<bool>())     c.vflip      = doc["vflip"];
  if (doc["led"].is<bool>())       c.led        = doc["led"];
  settingsSanitizeCamera(c);
  g_settings.camera = c;
  const bool applied = cameraApply(c);
  settingsSave();
  if (!applied && cameraIsReady()) sendMessage(cid, "error", "Some camera parameters could not be applied");
  sendConfig(0);
}

void handleCalibration(uint32_t cid, JsonDocument& doc) {
  JsonArray joints = doc["joints"];
  if (joints.isNull() || joints.size() != NUM_JOINTS) {
    sendMessage(cid, "error", "save_calibration requires 6 joints");
    return;
  }
  JointCalibration next[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    JsonObject j = joints[i];
    const JointCalibration& cur = g_settings.joints[i];
    next[i].channel    = j["channel"] | cur.channel;
    next[i].minPulseUs = j["minUs"]   | cur.minPulseUs;
    next[i].maxPulseUs = j["maxUs"]   | cur.maxPulseUs;
    next[i].minAngle   = j["min"]     | cur.minAngle;
    next[i].maxAngle   = j["max"]     | cur.maxAngle;
    next[i].homeAngle  = j["home"]    | cur.homeAngle;
    next[i].kinZero    = j["kinZero"] | cur.kinZero;
    next[i].kinDir     = (int)(j["kinDir"] | (int)cur.kinDir) < 0 ? -1 : 1;
    if (!settingsIsValidJoint(next[i])) {
      char msg[96];
      snprintf(msg, sizeof(msg), "Invalid calibration for joint %d (%s)", i, JOINT_NAMES[i]);
      sendMessage(cid, "error", msg);
      return;
    }
  }
  KinematicModel model = g_settings.model;
  JsonObject m = doc["model"];
  if (!m.isNull()) {
    model.L0 = m["L0"] | model.L0;
    model.L1 = m["L1"] | model.L1;
    model.L2 = m["L2"] | model.L2;
    model.L3 = m["L3"] | model.L3;
  }
  if (!settingsIsValidModel(model)) {
    sendMessage(cid, "error", "Invalid link lengths");
    return;
  }
  memcpy(g_settings.joints, next, sizeof(next));
  g_settings.model = model;
  reclampTargets();
  const bool saved = settingsSave();
  sendMessage(cid, saved ? "info" : "error", saved ? "Calibration saved" : "NVS write failed");
  sendConfig(0);
}

void handleHostname(uint32_t cid, JsonDocument& doc) {
  String name = doc["hostname"] | "";
  name.trim();
  name.toLowerCase();
  if (!settingsIsValidHostname(name.c_str())) {
    sendMessage(cid, "error", "Invalid hostname (use a-z, 0-9, '-' and '.', max 63 chars)");
    return;
  }
  strncpy(g_settings.hostname, name.c_str(), sizeof(g_settings.hostname) - 1);
  g_settings.hostname[sizeof(g_settings.hostname) - 1] = '\0';
  if (!settingsSave()) { sendMessage(cid, "error", "NVS write failed"); return; }
  sendMessage(cid, "info", "Hostname saved – restart the controller to apply");
  sendConfig(0);
}

void handleMessage(uint32_t cid, const char* text) {
  JsonDocument doc;
  if (deserializeJson(doc, text)) { sendMessage(cid, "error", "Invalid JSON"); return; }
  const char* type = doc["type"] | "";

  if (!strcmp(type, "emergency_stop")) {
    estopTrigger();       // already triggered in the WS callback; re-arm is harmless
    holdPosition();
    Serial.println("[ESTOP] Emergency stop triggered");
    broadcastStatus();
    return;
  }

  if (isMovementCommand(type, doc) && estopIsActive()) { sendLocked(cid); return; }

  if (!strcmp(type, "set_joint")) {
    const int id = doc["id"] | -1;
    if (id < 0 || id >= NUM_JOINTS || !doc["angle"].is<float>()) { sendMessage(cid, "error", "set_joint requires id 0-5 and angle"); return; }
    setJointAngle((uint8_t)id, doc["angle"].as<float>());
  } else if (!strcmp(type, "set_all")) {
    JsonArray arr = doc["angles"];
    if (arr.isNull() || arr.size() != NUM_JOINTS) { sendMessage(cid, "error", "set_all requires 6 angles"); return; }
    float a[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; ++i) a[i] = arr[i] | getJointAngle(i);
    setAllJointAngles(a);
  } else if (!strcmp(type, "home")) {
    moveHome();
  } else if (!strcmp(type, "ik_target")) {
    handleIK(cid, doc);
  } else if (!strcmp(type, "set_speed")) {
    const float v = doc["value"] | g_settings.maxSpeedDps;
    g_settings.maxSpeedDps = constrain(v, 5.0f, 360.0f);
    settingsSave();
    sendConfig(0);
  } else if (!strcmp(type, "camera")) {
    handleCamera(cid, doc);
  } else if (!strcmp(type, "save_calibration")) {
    handleCalibration(cid, doc);
  } else if (!strcmp(type, "set_hostname")) {
    handleHostname(cid, doc);
  } else if (!strcmp(type, "get_config")) {
    sendConfig(cid);
  } else if (!strcmp(type, "restart")) {
    sendMessage(0, "info", "Restarting controller…");
    s_restartAtMs = millis() + 500;
  } else if (!strcmp(type, "factory_reset")) {
    settingsFactoryReset();
    sendMessage(0, "info", "Factory defaults restored – restarting…");
    s_restartAtMs = millis() + 500;
  } else {
    sendMessage(cid, "error", "Unknown command");
  }
}
} // namespace

void controllerBegin() {
  s_lastTelemetryMs = millis();
}

void controllerLoop() {
  // Drain all queued WebSocket commands
  WsMessage msg;
  while (wifiNextMessage(msg)) {
    handleMessage(msg.clientId, msg.text);
    free(msg.text);
  }

  motionUpdate();

  const uint32_t now = millis();
  const bool estop = estopIsActive();
  if (estop != s_wasEstop) {          // push E-STOP edges immediately
    s_wasEstop = estop;
    if (!estop) Serial.println("[ESTOP] Lockout expired – movement enabled");
    broadcastStatus();
    s_lastTelemetryMs = now;
  } else if (now - s_lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    s_lastTelemetryMs = now;
    broadcastStatus();
  }

  if (s_restartAtMs && (int32_t)(now - s_restartAtMs) >= 0) {
    ESP.restart();
  }
}
