#include "settings.h"
#include <Arduino.h>
#include <Preferences.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>

RobotSettings g_settings;
StoredQuery   g_queries[MAX_STORED_QUERIES];

static_assert(sizeof(StoredQuery) == 30, "StoredQuery must be exactly 30 bytes");

namespace {
const char*    kNamespace       = "brazo";
const char*    kKey             = "cfg";
const char*    kQueriesKey      = "queries";
const char*    kQueriesMagicKey = "q_magic";
const uint32_t kMagic           = 0xB2A20002;   // bump when RobotSettings layout changes
const uint32_t kQueriesMagic    = 0xB2A25102;   // bump when StoredQuery layout changes

struct StoredBlob {
  uint32_t      magic;
  RobotSettings data;
};

int8_t clamp8(int v, int lo, int hi) { return (int8_t)(v < lo ? lo : (v > hi ? hi : v)); }

void clearQueriesMemory() {
  memset(g_queries, 0, sizeof(g_queries));
  for (int i = 0; i < MAX_STORED_QUERIES; ++i) {
    for (int j = 0; j < NUM_JOINTS; ++j) {
      g_queries[i].anglesTenths[j] = QUERY_WILDCARD_TENTHS;
      g_queries[i].speedsDps[j]    = QUERY_DEFAULT_SPEED;
    }
  }
}

// Trim leading and trailing whitespace in place
char* trimToken(char* s) {
  while (*s && isspace((unsigned char)*s)) ++s;
  char* end = s + strlen(s);
  while (end > s && isspace((unsigned char)*(end - 1))) --end;
  *end = '\0';
  return s;
}

bool parseSpeedValue(const char* tok, uint8_t& outSpd) {
  if (!tok || !*tok || strcmp(tok, "*") == 0) {
    outSpd = QUERY_DEFAULT_SPEED;
    return true;
  }
  char* endPtr = nullptr;
  const long val = strtol(tok, &endPtr, 10);
  if (endPtr == tok || *endPtr != '\0' || val < 0 || val > 255) return false;
  outSpd = (uint8_t)val;
  return true;
}
} // namespace

void settingsLoadDefaults() {
  memset(&g_settings, 0, sizeof(g_settings));
  strncpy(g_settings.hostname, DEFAULT_HOSTNAME, sizeof(g_settings.hostname) - 1);
  memcpy(g_settings.joints, DEFAULT_JOINT_CALIBRATIONS, sizeof(g_settings.joints));
  memcpy(g_settings.model.dh, DEFAULT_DH_PARAMS, sizeof(g_settings.model.dh));
  g_settings.maxSpeedDps = DEFAULT_MAX_SPEED_DPS;
  g_settings.camera = {
    (int8_t)DEFAULT_CAM_FRAMESIZE, (int8_t)DEFAULT_CAM_QUALITY,
    (int8_t)DEFAULT_CAM_BRIGHTNESS, (int8_t)DEFAULT_CAM_CONTRAST, (int8_t)DEFAULT_CAM_SATURATION,
    DEFAULT_CAM_HMIRROR != 0, DEFAULT_CAM_VFLIP != 0, DEFAULT_CAM_LED_STATE != 0
  };
  clearQueriesMemory();
}

void settingsBegin() {
  settingsLoadDefaults();
  Preferences prefs;
  if (!prefs.begin(kNamespace, true)) {
    Serial.println("[SETTINGS] No stored settings, using defaults");
    return;
  }
  StoredBlob blob;
  const size_t n = prefs.getBytesLength(kKey) == sizeof(blob) ? prefs.getBytes(kKey, &blob, sizeof(blob)) : 0;
  prefs.end();

  if (n != sizeof(blob) || blob.magic != kMagic) {
    Serial.println("[SETTINGS] Stored settings missing or outdated, using defaults");
  } else {
    // Validate before accepting – a corrupted blob must never move the arm out of range
    blob.data.hostname[sizeof(blob.data.hostname) - 1] = '\0';
    bool ok = settingsIsValidHostname(blob.data.hostname) && settingsIsValidModel(blob.data.model);
    for (int i = 0; i < NUM_JOINTS && ok; ++i) ok = settingsIsValidJoint(blob.data.joints[i]);
    if (!ok) {
      Serial.println("[SETTINGS] Stored settings invalid, using defaults");
    } else {
      g_settings = blob.data;
      if (!(g_settings.maxSpeedDps >= 5.0f && g_settings.maxSpeedDps <= 360.0f)) g_settings.maxSpeedDps = DEFAULT_MAX_SPEED_DPS;
      settingsSanitizeCamera(g_settings.camera);
      g_settings.camera.led = false; // Flash LED always starts OFF at boot
      Serial.printf("[SETTINGS] Loaded (hostname: %s)\n", g_settings.hostname);
    }
  }

  settingsLoadQueries();
}

bool settingsSave() {
  Preferences prefs;
  if (!prefs.begin(kNamespace, false)) return false;
  StoredBlob blob;
  blob.magic = kMagic;
  blob.data = g_settings;
  blob.data.camera.led = false; // Never persist flash LED ON state in NVS
  const size_t n = prefs.putBytes(kKey, &blob, sizeof(blob));
  prefs.end();
  return n == sizeof(blob);
}

void settingsFactoryReset() {
  Preferences prefs;
  if (prefs.begin(kNamespace, false)) {
    prefs.clear();
    prefs.end();
  }
  settingsLoadDefaults();
}

// Hostname: 1..63 chars, labels of [a-z0-9-] separated by dots, no leading/trailing '-' or '.'
bool settingsIsValidHostname(const char* name) {
  if (!name) return false;
  const size_t len = strlen(name);
  if (len == 0 || len > 63) return false;
  char prev = '.';
  for (size_t i = 0; i < len; ++i) {
    const char c = name[i];
    const bool alnum = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
    if (!(alnum || c == '-' || c == '.')) return false;
    if (c == '.' && (prev == '.' || prev == '-')) return false;
    if (c == '-' && prev == '.') return false;
    prev = c;
  }
  return prev != '.' && prev != '-';
}

bool settingsIsValidJoint(const JointCalibration& j) {
  return j.channel < 16 &&
         j.minPulseUs >= 300 && j.maxPulseUs <= 2800 && j.minPulseUs < j.maxPulseUs &&
         j.minAngle >= 0.0f && j.maxAngle <= 360.0f && j.minAngle < j.maxAngle &&
         j.homeAngle >= j.minAngle && j.homeAngle <= j.maxAngle &&
         j.kinZero >= 0.0f && j.kinZero <= 360.0f &&
         (j.kinDir == 1 || j.kinDir == -1);
}

bool settingsIsValidModel(const KinematicModel& m) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    const DHParam& d = m.dh[i];
    if (!isfinite(d.alpha) || !isfinite(d.a) || !isfinite(d.d) || !isfinite(d.theta)) return false;
    if (d.alpha < -360.0f || d.alpha > 360.0f) return false;
    if (d.a < -1000.0f || d.a > 1000.0f) return false;
    if (d.d < -1000.0f || d.d > 1000.0f) return false;
    if (d.theta < -360.0f || d.theta > 360.0f) return false;
  }
  return true;
}

void settingsSanitizeCamera(CameraSettings& c) {
  c.framesize  = clamp8(c.framesize, 1, 13);
  c.quality    = clamp8(c.quality, 4, 63);
  c.brightness = clamp8(c.brightness, -2, 2);
  c.contrast   = clamp8(c.contrast, -2, 2);
  c.saturation = clamp8(c.saturation, -2, 2);
}

// Query name: 1..10 chars, alphanumeric (A-Z, a-z, 0-9) plus '_' or '-'
bool settingsIsValidQueryName(const char* name) {
  if (!name) return false;
  const size_t len = strlen(name);
  if (len == 0 || len > QUERY_NAME_MAX_LEN) return false;
  for (size_t i = 0; i < len; ++i) {
    const char c = name[i];
    const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                    (c >= '0' && c <= '9') || c == '_' || c == '-';
    if (!ok) return false;
  }
  return true;
}

bool settingsIsValidStoredQuery(const StoredQuery& q) {
  if (!q.used) return true;
  if (!settingsIsValidQueryName(q.name)) return false;
  bool anyActive = false;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    const int16_t t = q.anglesTenths[i];
    if (t == QUERY_WILDCARD_TENTHS) continue;
    anyActive = true;
    const int16_t minT = (int16_t)lroundf(g_settings.joints[i].minAngle * 10.0f);
    const int16_t maxT = (int16_t)lroundf(g_settings.joints[i].maxAngle * 10.0f);
    if (t < minT || t > maxT) return false;
  }
  return anyActive;
}

bool settingsLoadQueries() {
  clearQueriesMemory();
  Preferences prefs;
  if (!prefs.begin(kNamespace, true)) return false;
  const uint32_t magic = prefs.getUInt(kQueriesMagicKey, 0);
  const size_t n = (magic == kQueriesMagic && prefs.getBytesLength(kQueriesKey) == sizeof(g_queries))
                     ? prefs.getBytes(kQueriesKey, g_queries, sizeof(g_queries))
                     : 0;
  prefs.end();

  if (n != sizeof(g_queries)) {
    clearQueriesMemory();
    Serial.println("[SETTINGS] No stored movement queries found (starting empty)");
    return false;
  }

  int loaded = 0;
  for (int i = 0; i < MAX_STORED_QUERIES; ++i) {
    StoredQuery& q = g_queries[i];
    q.name[QUERY_NAME_MAX_LEN] = '\0';
    if (q.used && settingsIsValidStoredQuery(q)) {
      ++loaded;
    } else {
      memset(&q, 0, sizeof(StoredQuery));
      for (int j = 0; j < NUM_JOINTS; ++j) {
        q.anglesTenths[j] = QUERY_WILDCARD_TENTHS;
        q.speedsDps[j]    = QUERY_DEFAULT_SPEED;
      }
    }
  }
  Serial.printf("[SETTINGS] Loaded %d/%d movement queries (%u bytes reserved)\n",
                loaded, MAX_STORED_QUERIES, (unsigned)(sizeof(uint32_t) + sizeof(g_queries)));
  return true;
}

bool settingsSaveQueries() {
  Preferences prefs;
  if (!prefs.begin(kNamespace, false)) return false;
  const bool okMagic = prefs.putUInt(kQueriesMagicKey, kQueriesMagic) == sizeof(uint32_t);
  const size_t n = prefs.putBytes(kQueriesKey, g_queries, sizeof(g_queries));
  prefs.end();
  return okMagic && (n == sizeof(g_queries));
}

int settingsCountQueries() {
  int count = 0;
  for (int i = 0; i < MAX_STORED_QUERIES; ++i) {
    if (g_queries[i].used) ++count;
  }
  return count;
}

bool settingsParseQuery(const char* anglesStr, const char* speedsStr,
                        int16_t outTenths[NUM_JOINTS], uint8_t outSpeeds[NUM_JOINTS],
                        char* errBuf, size_t errLen) {
  if (errBuf && errLen > 0) errBuf[0] = '\0';
  if (!anglesStr || !*anglesStr) {
    if (errBuf) snprintf(errBuf, errLen, "Query string is empty");
    return false;
  }

  char aBuf[160];
  strncpy(aBuf, anglesStr, sizeof(aBuf) - 1);
  aBuf[sizeof(aBuf) - 1] = '\0';

  bool hasInlineSpeed[NUM_JOINTS] = {false};
  int  tokenCount = 0;
  bool anyActive  = false;

  char* savePtr = nullptr;
  for (char* raw = strtok_r(aBuf, ",", &savePtr); raw != nullptr; raw = strtok_r(nullptr, ",", &savePtr)) {
    if (tokenCount >= NUM_JOINTS) {
      if (errBuf) snprintf(errBuf, errLen, "Expected 6 joint values (J0..J5), got more than 6");
      return false;
    }
    const int idx = tokenCount++;
    char* tok = trimToken(raw);
    outSpeeds[idx] = QUERY_DEFAULT_SPEED;

    // Check for inline speed separator '@' or ':'
    char* sep = strchr(tok, '@');
    if (!sep) sep = strchr(tok, ':');
    if (sep) {
      *sep = '\0';
      char* spdTok = trimToken(sep + 1);
      tok = trimToken(tok);
      uint8_t spd = QUERY_DEFAULT_SPEED;
      if (!parseSpeedValue(spdTok, spd)) {
        if (errBuf) snprintf(errBuf, errLen, "J%d (%s) speed must be 0..255 deg/s or *", idx, JOINT_NAMES[idx]);
        return false;
      }
      outSpeeds[idx] = spd;
      hasInlineSpeed[idx] = true;
    }

    if (strcmp(tok, "*") == 0) {
      outTenths[idx] = QUERY_WILDCARD_TENTHS;
      outSpeeds[idx] = QUERY_DEFAULT_SPEED;
      continue;
    }

    char* endPtr = nullptr;
    const float deg = strtof(tok, &endPtr);
    if (endPtr == tok || *endPtr != '\0' || !isfinite(deg)) {
      if (errBuf) snprintf(errBuf, errLen, "J%d (%s): invalid angle '%s' (use degrees or *)", idx, JOINT_NAMES[idx], tok);
      return false;
    }

    const JointCalibration& cal = g_settings.joints[idx];
    if (deg < cal.minAngle - 0.05f || deg > cal.maxAngle + 0.05f) {
      if (errBuf) snprintf(errBuf, errLen, "J%d (%s) angle %.1f out of limits (%.1f..%.1f deg)",
                           idx, JOINT_NAMES[idx], deg, cal.minAngle, cal.maxAngle);
      return false;
    }

    const float clamped = constrain(deg, cal.minAngle, cal.maxAngle);
    outTenths[idx] = (int16_t)lroundf(clamped * 10.0f);
    anyActive = true;
  }

  if (tokenCount != NUM_JOINTS) {
    if (errBuf) snprintf(errBuf, errLen, "Expected 6 comma-separated values (J0..J5), got %d", tokenCount);
    return false;
  }
  if (!anyActive) {
    if (errBuf) snprintf(errBuf, errLen, "At least one joint must have a target angle (not all '*')");
    return false;
  }

  // Parse optional companion speedsStr (either a single value for all moving joints, or 6 comma-separated values)
  if (speedsStr && *speedsStr) {
    char sBuf[128];
    strncpy(sBuf, speedsStr, sizeof(sBuf) - 1);
    sBuf[sizeof(sBuf) - 1] = '\0';
    char* trimmedAll = trimToken(sBuf);
    if (*trimmedAll) {
      if (!strchr(trimmedAll, ',')) {
        uint8_t globalSpd = QUERY_DEFAULT_SPEED;
        if (!parseSpeedValue(trimmedAll, globalSpd)) {
          if (errBuf) snprintf(errBuf, errLen, "Speed must be 0..255 deg/s or *");
          return false;
        }
        for (int i = 0; i < NUM_JOINTS; ++i) {
          if (outTenths[i] != QUERY_WILDCARD_TENTHS && !hasInlineSpeed[i]) outSpeeds[i] = globalSpd;
        }
      } else {
        int sCount = 0;
        char* sSave = nullptr;
        for (char* sRaw = strtok_r(trimmedAll, ",", &sSave); sRaw != nullptr; sRaw = strtok_r(nullptr, ",", &sSave)) {
          if (sCount >= NUM_JOINTS) {
            if (errBuf) snprintf(errBuf, errLen, "Expected 6 speed values (J0..J5), got more than 6");
            return false;
          }
          const int idx = sCount++;
          char* sTok = trimToken(sRaw);
          uint8_t spd = QUERY_DEFAULT_SPEED;
          if (!parseSpeedValue(sTok, spd)) {
            if (errBuf) snprintf(errBuf, errLen, "J%d (%s) speed must be 0..255 deg/s or *", idx, JOINT_NAMES[idx]);
            return false;
          }
          if (outTenths[idx] != QUERY_WILDCARD_TENTHS && !hasInlineSpeed[idx]) {
            outSpeeds[idx] = spd;
          }
        }
        if (sCount != NUM_JOINTS) {
          if (errBuf) snprintf(errBuf, errLen, "Speeds field requires either 1 global value or 6 comma-separated values (got %d)", sCount);
          return false;
        }
      }
    }
  }

  return true;
}

void settingsFormatQueryAngles(const int16_t tenths[NUM_JOINTS], char* outBuf, size_t outLen) {
  if (!outBuf || outLen == 0) return;
  outBuf[0] = '\0';
  size_t pos = 0;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    char tok[16];
    if (tenths[i] == QUERY_WILDCARD_TENTHS) {
      strcpy(tok, "*");
    } else if (tenths[i] % 10 == 0) {
      snprintf(tok, sizeof(tok), "%d", (int)(tenths[i] / 10));
    } else {
      snprintf(tok, sizeof(tok), "%.1f", tenths[i] * 0.1f);
    }
    const int n = snprintf(outBuf + pos, outLen - pos, "%s%s", (i > 0 ? ", " : ""), tok);
    if (n < 0 || (size_t)n >= outLen - pos) break;
    pos += (size_t)n;
  }
}

void settingsFormatQuerySpeeds(const uint8_t speeds[NUM_JOINTS], char* outBuf, size_t outLen) {
  if (!outBuf || outLen == 0) return;
  outBuf[0] = '\0';
  size_t pos = 0;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    char tok[8];
    if (speeds[i] == QUERY_DEFAULT_SPEED) {
      strcpy(tok, "*");
    } else {
      snprintf(tok, sizeof(tok), "%u", (unsigned)speeds[i]);
    }
    const int n = snprintf(outBuf + pos, outLen - pos, "%s%s", (i > 0 ? ", " : ""), tok);
    if (n < 0 || (size_t)n >= outLen - pos) break;
    pos += (size_t)n;
  }
}
