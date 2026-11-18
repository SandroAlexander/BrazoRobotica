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

// One-by-one (sequential) query execution state
bool     s_seqActive = false;
uint8_t  s_seqNextJoint = 0;
int8_t   s_seqActiveJoint = -1;
int16_t  s_seqTargetsTenths[NUM_JOINTS];
uint8_t  s_seqSpeedsDps[NUM_JOINTS];

void cancelSequential() {
  s_seqActive = false;
  s_seqNextJoint = 0;
  s_seqActiveJoint = -1;
}

bool advanceSequential() {
  while (s_seqNextJoint < NUM_JOINTS) {
    const uint8_t j = s_seqNextJoint++;
    if (s_seqTargetsTenths[j] == QUERY_WILDCARD_TENTHS) continue;
    const float targetDeg = s_seqTargetsTenths[j] * 0.1f;
    const float curDeg = getJointAngle(j);
    setJointAngleWithSpeed(j, targetDeg, s_seqSpeedsDps[j]);
    if (fabsf(targetDeg - curDeg) > 0.05f) {
      s_seqActiveJoint = (int8_t)j;
      return true;
    }
  }
  cancelSequential();
  return false;
}

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

  JsonArray dhArr = doc["dh"].to<JsonArray>();
  for (int i = 0; i < NUM_JOINTS; ++i) {
    const DHParam& d = g_settings.model.dh[i];
    JsonObject row = dhArr.add<JsonObject>();
    row["alpha"] = f1(d.alpha);
    row["a"]     = f1(d.a);
    row["d"]     = f1(d.d);
    row["theta"] = f1(d.theta);
  }

  JsonObject ikDef = doc["ikDefaults"].to<JsonObject>();
  ikDef["iterations"] = IK_DEFAULT_ITERATIONS;
  ikDef["population"] = IK_DEFAULT_POPULATION;

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

void sendQueries(uint32_t cid) {
  JsonDocument doc;
  doc["type"] = "queries";
  doc["max"] = MAX_STORED_QUERIES;
  doc["bytesPerQuery"] = (unsigned)sizeof(StoredQuery);
  doc["reservedBytes"] = (unsigned)(sizeof(uint32_t) + sizeof(g_queries));
  doc["usedCount"] = settingsCountQueries();
  JsonArray items = doc["items"].to<JsonArray>();
  for (int i = 0; i < MAX_STORED_QUERIES; ++i) {
    if (!g_queries[i].used) continue;
    JsonObject q = items.add<JsonObject>();
    q["id"] = i;
    q["name"] = g_queries[i].name;
    char aStr[96], sStr[64];
    settingsFormatQueryAngles(g_queries[i].anglesTenths, aStr, sizeof(aStr));
    settingsFormatQuerySpeeds(g_queries[i].speedsDps, sStr, sizeof(sStr));
    q["angles"] = aStr;
    q["speeds"] = sStr;
  }
  sendJson(cid, doc);
}

void sendConfig(uint32_t cid) {
  JsonDocument doc;
  buildConfig(doc);
  sendJson(cid, doc);
  sendQueries(cid);
}

void broadcastStatus() {
  float cur[NUM_JOINTS], tgt[NUM_JOINTS], tip[3], rel[NUM_JOINTS][3];
  getJointAngles(cur);
  getTargetAngles(tgt);
  ikForward(cur, g_settings.joints, g_settings.model, tip, nullptr, rel);
  const uint32_t remaining = estopRemainingMs();
  const bool movingNow = isMoving() || s_seqActive;

  char buf[880];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"status\",\"angles\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
    "\"targets\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],\"tip\":[%.1f,%.1f,%.1f],"
    "\"rel\":[[%.1f,%.1f,%.1f],[%.1f,%.1f,%.1f],[%.1f,%.1f,%.1f],[%.1f,%.1f,%.1f],[%.1f,%.1f,%.1f],[%.1f,%.1f,%.1f]],"
    "\"moving\":%s,\"seq\":%s,\"seq_joint\":%d,"
    "\"estop\":%s,\"estop_ms\":%u,\"pwm\":%s,\"camera\":%s,\"streaming\":%s,"
    "\"uptime_ms\":%lu,\"clients\":%u,\"heap\":%u}",
    cur[0], cur[1], cur[2], cur[3], cur[4], cur[5],
    tgt[0], tgt[1], tgt[2], tgt[3], tgt[4], tgt[5],
    tip[0], tip[1], tip[2],
    rel[0][0], rel[0][1], rel[0][2],
    rel[1][0], rel[1][1], rel[1][2],
    rel[2][0], rel[2][1], rel[2][2],
    rel[3][0], rel[3][1], rel[3][2],
    rel[4][0], rel[4][1], rel[4][2],
    rel[5][0], rel[5][1], rel[5][2],
    movingNow ? "true" : "false", s_seqActive ? "true" : "false", (int)s_seqActiveJoint,
    remaining ? "true" : "false", (unsigned)remaining,
    pwmIsOnline() ? "true" : "false", cameraIsReady() ? "true" : "false",
    cameraIsStreaming() ? "true" : "false",
    (unsigned long)millis(), (unsigned)wsClientCount(), (unsigned)ESP.getFreeHeap());
  wsBroadcast(buf);
}

bool isMovementCommand(const char* type, JsonDocument& doc) {
  if (!strcmp(type, "set_joint") || !strcmp(type, "set_all") || !strcmp(type, "home") || !strcmp(type, "run_query")) return true;
  if (!strcmp(type, "ik_target")) return doc["move"] | true;
  return false;
}

// Forward declaration for sequential / simultaneous execution helper
void executeQueryTargets(uint32_t cid, const char* label, const int16_t tenths[NUM_JOINTS],
                         const uint8_t speeds[NUM_JOINTS], bool sequential);

bool parseDHArray(JsonArray arr, KinematicModel& outModel) {
  if (arr.isNull() || arr.size() != NUM_JOINTS) return false;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    JsonObject row = arr[i];
    if (row.isNull()) return false;
    const DHParam& cur = g_settings.model.dh[i];
    outModel.dh[i].alpha = row["alpha"] | cur.alpha;
    outModel.dh[i].a     = row["a"]     | cur.a;
    outModel.dh[i].d     = row["d"]     | cur.d;
    outModel.dh[i].theta = row["theta"] | cur.theta;
  }
  return settingsIsValidModel(outModel);
}

// ------------------------------------------------------------------ handlers
void handleIK(uint32_t cid, JsonDocument& doc) {
  if (!doc["x"].is<float>() || !doc["y"].is<float>() || !doc["z"].is<float>()) {
    sendMessage(cid, "error", "ik_target requires numeric x, y, z");
    return;
  }
  IKTarget t;
  t.x              = doc["x"];
  t.y              = doc["y"];
  t.z              = doc["z"];
  t.useOrientation = doc["orient"] | false;
  t.pitchDeg       = doc["pitch"] | -90.0f;
  t.iterations     = doc["iterations"] | IK_DEFAULT_ITERATIONS;
  t.population     = doc["population"] | IK_DEFAULT_POPULATION;
  t.lockGripper    = doc["lockGripper"] | true;

  const bool move = doc["move"] | true;
  const char* mode = doc["mode"] | "all";
  const bool sequential = (strcmp(mode, "seq") == 0 || strcmp(mode, "one_by_one") == 0);

  float seed[NUM_JOINTS];
  getTargetAngles(seed);   // continue from where the arm is heading

  const uint32_t t0 = micros();
  const IKResult r = ikSolve(t, seed, g_settings.joints, g_settings.model);
  const uint32_t dt = micros() - t0;

  // Differential Evolution returns the member with the closest distance to the target point.
  const bool moved = move && !estopIsActive();
  if (moved) {
    if (sequential) {
      int16_t tenths[NUM_JOINTS];
      uint8_t speeds[NUM_JOINTS];
      for (int i = 0; i < NUM_JOINTS; ++i) {
        if (t.lockGripper && i == 5) {
          tenths[i] = QUERY_WILDCARD_TENTHS;
        } else {
          tenths[i] = (int16_t)lroundf(r.angles[i] * 10.0f);
        }
        speeds[i] = QUERY_DEFAULT_SPEED;
      }
      executeQueryTargets(cid, "IK solution", tenths, speeds, true);
    } else {
      cancelSequential();
      setAllJointAngles(r.angles);
      broadcastStatus();
    }
  }

  JsonDocument out;
  out["type"] = "ik_result";
  out["success"] = r.success;
  out["moved"] = moved;
  out["mode"] = sequential ? "seq" : "all";
  out["iterations"] = r.iterations;
  out["population"] = r.population;
  out["posErr"] = f1(r.positionError);
  out["oriErr"] = serialized(String(r.orientationError, 3));
  out["time_us"] = dt;
  JsonArray a = out["angles"].to<JsonArray>();
  for (int i = 0; i < NUM_JOINTS; ++i) a.add(f1(r.angles[i]));
  sendJson(cid, out);
}

void handleSaveDH(uint32_t cid, JsonDocument& doc) {
  KinematicModel model = g_settings.model;
  if (!parseDHArray(doc["dh"], model)) {
    sendMessage(cid, "error", "Invalid Denavit-Hartenberg parameters (6 joints required, angles -360..360, lengths -1000..1000 mm)");
    return;
  }
  g_settings.model = model;
  const bool saved = settingsSave();
  sendMessage(cid, saved ? "info" : "error", saved ? "Denavit-Hartenberg parameters saved to NVS" : "NVS write failed");
  sendConfig(0);
  broadcastStatus();
}

void handleResetDH(uint32_t cid) {
  memcpy(g_settings.model.dh, DEFAULT_DH_PARAMS, sizeof(g_settings.model.dh));
  const bool saved = settingsSave();
  sendMessage(cid, saved ? "info" : "error", saved ? "Default Denavit-Hartenberg parameters restored" : "NVS write failed");
  sendConfig(0);
  broadcastStatus();
}

void handleCamera(uint32_t cid, JsonDocument& doc) {
  CameraSettings c = g_settings.camera;
  bool sensorChanged = false;
  if (doc["framesize"].is<int>())  { c.framesize  = doc["framesize"];  sensorChanged = true; }
  if (doc["quality"].is<int>())    { c.quality    = doc["quality"];    sensorChanged = true; }
  if (doc["brightness"].is<int>()) { c.brightness = doc["brightness"]; sensorChanged = true; }
  if (doc["contrast"].is<int>())   { c.contrast   = doc["contrast"];   sensorChanged = true; }
  if (doc["saturation"].is<int>()) { c.saturation = doc["saturation"]; sensorChanged = true; }
  if (doc["hmirror"].is<bool>())   { c.hmirror    = doc["hmirror"];    sensorChanged = true; }
  if (doc["vflip"].is<bool>())     { c.vflip      = doc["vflip"];      sensorChanged = true; }
  if (doc["led"].is<bool>())       { c.led        = doc["led"]; }
  settingsSanitizeCamera(c);
  g_settings.camera = c;
  if (sensorChanged) {
    const bool applied = cameraApply(c);
    settingsSave();
    if (!applied && cameraIsReady()) sendMessage(cid, "error", "Some camera parameters could not be applied");
  } else {
    cameraSetLed(c.led);
  }
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
  if (!doc["dh"].isNull()) {
    if (!parseDHArray(doc["dh"], model)) {
      sendMessage(cid, "error", "Invalid Denavit-Hartenberg parameters");
      return;
    }
  }
  memcpy(g_settings.joints, next, sizeof(next));
  g_settings.model = model;
  reclampTargets();
  const bool saved = settingsSave();
  sendMessage(cid, saved ? "info" : "error", saved ? "Calibration saved" : "NVS write failed");
  sendConfig(0);
  broadcastStatus();
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

void executeQueryTargets(uint32_t cid, const char* label, const int16_t tenths[NUM_JOINTS],
                         const uint8_t speeds[NUM_JOINTS], bool sequential) {
  cancelSequential();
  if (sequential) {
    holdPosition();
    memcpy(s_seqTargetsTenths, tenths, sizeof(s_seqTargetsTenths));
    memcpy(s_seqSpeedsDps, speeds, sizeof(s_seqSpeedsDps));
    s_seqActive = true;
    s_seqNextJoint = 0;
    if (!advanceSequential()) {
      sendMessage(cid, "info", "Arm is already at the requested query position");
      return;
    }
    char msg[80];
    snprintf(msg, sizeof(msg), "Executing %s one-by-one (J%d first)…", label, (int)s_seqActiveJoint);
    sendMessage(cid, "info", msg);
  } else {
    float targets[NUM_JOINTS];
    getJointAngles(targets); // '*' preserves the servo's current angle
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (tenths[i] != QUERY_WILDCARD_TENTHS) {
        targets[i] = tenths[i] * 0.1f;
      }
    }
    setAllJointAnglesWithSpeeds(targets, speeds);
    char msg[80];
    snprintf(msg, sizeof(msg), "Executing %s (all servos simultaneously)", label);
    sendMessage(cid, "info", msg);
  }
  broadcastStatus();
}

void handleRunQuery(uint32_t cid, JsonDocument& doc) {
  const char* mode = doc["mode"] | "all";
  const bool sequential = (strcmp(mode, "seq") == 0 || strcmp(mode, "one_by_one") == 0);

  if (doc["id"].is<int>() && !doc["angles"].is<const char*>() && !doc["query"].is<const char*>()) {
    const int id = doc["id"];
    if (id < 0 || id >= MAX_STORED_QUERIES || !g_queries[id].used) {
      sendMessage(cid, "error", "Invalid or empty query slot");
      return;
    }
    executeQueryTargets(cid, g_queries[id].name, g_queries[id].anglesTenths, g_queries[id].speedsDps, sequential);
    return;
  }

  const char* anglesStr = doc["angles"] | (const char*)(doc["query"] | "");
  const char* speedsStr = doc["speeds"] | "";
  int16_t tenths[NUM_JOINTS];
  uint8_t speeds[NUM_JOINTS];
  char err[128];
  if (!settingsParseQuery(anglesStr, speedsStr, tenths, speeds, err, sizeof(err))) {
    sendMessage(cid, "error", err);
    return;
  }
  executeQueryTargets(cid, "query", tenths, speeds, sequential);
}

void handleSaveQuery(uint32_t cid, JsonDocument& doc) {
  String name = doc["name"] | "";
  name.trim();
  if (!settingsIsValidQueryName(name.c_str())) {
    sendMessage(cid, "error", "Name must be 1..10 alphanumeric characters (A-Z, 0-9, _, -)");
    return;
  }

  const char* anglesStr = doc["angles"] | (const char*)(doc["query"] | "");
  const char* speedsStr = doc["speeds"] | "";
  int16_t tenths[NUM_JOINTS];
  uint8_t speeds[NUM_JOINTS];
  char err[128];
  if (!settingsParseQuery(anglesStr, speedsStr, tenths, speeds, err, sizeof(err))) {
    sendMessage(cid, "error", err);
    return;
  }

  int slot = -1;
  if (doc["id"].is<int>()) {
    const int reqId = doc["id"];
    if (reqId >= 0 && reqId < MAX_STORED_QUERIES) slot = reqId;
  }
  if (slot < 0) {
    // Update existing query with same name (case-insensitive) if present
    for (int i = 0; i < MAX_STORED_QUERIES; ++i) {
      if (g_queries[i].used && strcasecmp(g_queries[i].name, name.c_str()) == 0) {
        slot = i;
        break;
      }
    }
  }
  if (slot < 0) {
    // Allocate first free slot
    for (int i = 0; i < MAX_STORED_QUERIES; ++i) {
      if (!g_queries[i].used) {
        slot = i;
        break;
      }
    }
  }
  if (slot < 0) {
    sendMessage(cid, "error", "Query storage full (50/50 slots used)");
    return;
  }

  StoredQuery& q = g_queries[slot];
  memset(&q, 0, sizeof(q));
  strncpy(q.name, name.c_str(), QUERY_NAME_MAX_LEN);
  q.name[QUERY_NAME_MAX_LEN] = '\0';
  q.used = 1;
  memcpy(q.anglesTenths, tenths, sizeof(tenths));
  memcpy(q.speedsDps, speeds, sizeof(speeds));

  if (!settingsSaveQueries()) {
    sendMessage(cid, "error", "NVS write failed while saving query");
    return;
  }

  char msg[64];
  snprintf(msg, sizeof(msg), "Query \"%s\" saved in slot #%02d", q.name, slot + 1);
  sendMessage(cid, "info", msg);
  sendQueries(0);
}

void handleDeleteQuery(uint32_t cid, JsonDocument& doc) {
  const int id = doc["id"] | -1;
  if (id < 0 || id >= MAX_STORED_QUERIES || !g_queries[id].used) {
    sendMessage(cid, "error", "Query slot not found");
    return;
  }
  char oldName[QUERY_NAME_MAX_LEN + 1];
  strncpy(oldName, g_queries[id].name, sizeof(oldName));
  oldName[QUERY_NAME_MAX_LEN] = '\0';

  memset(&g_queries[id], 0, sizeof(StoredQuery));
  for (int j = 0; j < NUM_JOINTS; ++j) {
    g_queries[id].anglesTenths[j] = QUERY_WILDCARD_TENTHS;
    g_queries[id].speedsDps[j]    = QUERY_DEFAULT_SPEED;
  }
  settingsSaveQueries();

  char msg[64];
  snprintf(msg, sizeof(msg), "Deleted query \"%s\"", oldName);
  sendMessage(cid, "info", msg);
  sendQueries(0);
}

void handleMessage(uint32_t cid, const char* text) {
  JsonDocument doc;
  if (deserializeJson(doc, text)) { sendMessage(cid, "error", "Invalid JSON"); return; }
  const char* type = doc["type"] | "";

  if (!strcmp(type, "emergency_stop")) {
    cancelSequential();
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
    cancelSequential();
    setJointAngle((uint8_t)id, doc["angle"].as<float>());
  } else if (!strcmp(type, "set_all")) {
    JsonArray arr = doc["angles"];
    if (arr.isNull() || arr.size() != NUM_JOINTS) { sendMessage(cid, "error", "set_all requires 6 angles"); return; }
    float a[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; ++i) a[i] = arr[i] | getJointAngle(i);
    cancelSequential();
    setAllJointAngles(a);
  } else if (!strcmp(type, "home")) {
    cancelSequential();
    moveHome();
  } else if (!strcmp(type, "ik_target")) {
    handleIK(cid, doc);
  } else if (!strcmp(type, "save_dh")) {
    handleSaveDH(cid, doc);
  } else if (!strcmp(type, "reset_dh")) {
    handleResetDH(cid);
  } else if (!strcmp(type, "run_query")) {
    handleRunQuery(cid, doc);
  } else if (!strcmp(type, "save_query")) {
    handleSaveQuery(cid, doc);
  } else if (!strcmp(type, "delete_query")) {
    handleDeleteQuery(cid, doc);
  } else if (!strcmp(type, "get_queries")) {
    sendQueries(cid);
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
    cancelSequential();
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

  if (estopIsActive()) {
    cancelSequential();
  } else if (s_seqActive && !isMoving()) {
    advanceSequential();
  }

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
