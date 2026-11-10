#ifndef SETTINGS_H
#define SETTINGS_H

// =====================================================================
//  Runtime settings persisted in NVS (ESP32 Preferences)
// =====================================================================

#include "config.h"
#include "ik_solver.h"

struct CameraSettings {
  int8_t framesize;    // framesize_t value
  int8_t quality;      // 4..63
  int8_t brightness;   // -2..2
  int8_t contrast;     // -2..2
  int8_t saturation;   // -2..2
  bool   hmirror;
  bool   vflip;
  bool   led;
};

struct RobotSettings {
  char             hostname[64];
  JointCalibration joints[NUM_JOINTS];
  KinematicModel   model;
  float            maxSpeedDps;
  CameraSettings   camera;
};

extern RobotSettings g_settings;

void settingsBegin();               // load from NVS (or defaults)
bool settingsSave();                // persist current g_settings
void settingsLoadDefaults();        // factory values into g_settings (not saved)
void settingsFactoryReset();        // erase NVS + load defaults

bool settingsIsValidHostname(const char* name);
bool settingsIsValidJoint(const JointCalibration& j);
bool settingsIsValidModel(const KinematicModel& m);
void settingsSanitizeCamera(CameraSettings& c);

#endif // SETTINGS_H
