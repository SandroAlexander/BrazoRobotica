#include "settings.h"
#include <Arduino.h>
#include <Preferences.h>
#include <string.h>
#include <ctype.h>

RobotSettings g_settings;

namespace {
const char*    kNamespace = "brazo";
const char*    kKey       = "cfg";
const uint32_t kMagic     = 0xB2A20001;   // bump when RobotSettings layout changes

struct StoredBlob {
  uint32_t      magic;
  RobotSettings data;
};

int8_t clamp8(int v, int lo, int hi) { return (int8_t)(v < lo ? lo : (v > hi ? hi : v)); }
} // namespace

void settingsLoadDefaults() {
  memset(&g_settings, 0, sizeof(g_settings));
  strncpy(g_settings.hostname, DEFAULT_HOSTNAME, sizeof(g_settings.hostname) - 1);
  memcpy(g_settings.joints, DEFAULT_JOINT_CALIBRATIONS, sizeof(g_settings.joints));
  g_settings.model = {DEFAULT_LINK_L0, DEFAULT_LINK_L1, DEFAULT_LINK_L2, DEFAULT_LINK_L3};
  g_settings.maxSpeedDps = DEFAULT_MAX_SPEED_DPS;
  g_settings.camera = {
    (int8_t)DEFAULT_CAM_FRAMESIZE, (int8_t)DEFAULT_CAM_QUALITY,
    (int8_t)DEFAULT_CAM_BRIGHTNESS, (int8_t)DEFAULT_CAM_CONTRAST, (int8_t)DEFAULT_CAM_SATURATION,
    DEFAULT_CAM_HMIRROR != 0, DEFAULT_CAM_VFLIP != 0, DEFAULT_CAM_LED_STATE != 0
  };
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
    return;
  }

  // Validate before accepting – a corrupted blob must never move the arm out of range
  blob.data.hostname[sizeof(blob.data.hostname) - 1] = '\0';
  bool ok = settingsIsValidHostname(blob.data.hostname) && settingsIsValidModel(blob.data.model);
  for (int i = 0; i < NUM_JOINTS && ok; ++i) ok = settingsIsValidJoint(blob.data.joints[i]);
  if (!ok) {
    Serial.println("[SETTINGS] Stored settings invalid, using defaults");
    return;
  }
  g_settings = blob.data;
  if (!(g_settings.maxSpeedDps >= 5.0f && g_settings.maxSpeedDps <= 360.0f)) g_settings.maxSpeedDps = DEFAULT_MAX_SPEED_DPS;
  settingsSanitizeCamera(g_settings.camera);
  Serial.printf("[SETTINGS] Loaded (hostname: %s)\n", g_settings.hostname);
}

bool settingsSave() {
  Preferences prefs;
  if (!prefs.begin(kNamespace, false)) return false;
  StoredBlob blob;
  blob.magic = kMagic;
  blob.data = g_settings;
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
  return m.L0 >= 0.0f && m.L0 <= 1000.0f &&
         m.L1 > 0.0f && m.L1 <= 1000.0f &&
         m.L2 > 0.0f && m.L2 <= 1000.0f &&
         m.L3 >= 0.0f && m.L3 <= 1000.0f;
}

void settingsSanitizeCamera(CameraSettings& c) {
  c.framesize  = clamp8(c.framesize, 1, 13);
  c.quality    = clamp8(c.quality, 4, 63);
  c.brightness = clamp8(c.brightness, -2, 2);
  c.contrast   = clamp8(c.contrast, -2, 2);
  c.saturation = clamp8(c.saturation, -2, 2);
}
