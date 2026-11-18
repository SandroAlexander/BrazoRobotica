#ifndef SETTINGS_H
#define SETTINGS_H

// =====================================================================
//  Runtime settings persisted in NVS (ESP32 Preferences)
// =====================================================================

#include <stddef.h>
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

// Compact 30-byte stored movement query:
//   name         : 11 bytes (up to 10 alphanumeric chars + '\0')
//   used         : 1 byte   (1 = occupied slot, 0 = empty)
//   anglesTenths : 12 bytes (6 x int16_t: angle * 10 in 0..3600, or -1 for '*')
//   speedsDps    : 6 bytes  (6 x uint8_t: 0 = global default speed, 1..255 = °/s)
struct StoredQuery {
  char    name[QUERY_NAME_MAX_LEN + 1];
  uint8_t used;
  int16_t anglesTenths[NUM_JOINTS];
  uint8_t speedsDps[NUM_JOINTS];
};

extern RobotSettings g_settings;
extern StoredQuery   g_queries[MAX_STORED_QUERIES];

void settingsBegin();               // load settings & queries from NVS (or defaults)
bool settingsSave();                // persist current g_settings
void settingsLoadDefaults();        // factory values into g_settings & g_queries (not saved)
void settingsFactoryReset();        // erase NVS + load defaults

bool settingsLoadQueries();         // load g_queries from NVS key "queries"
bool settingsSaveQueries();         // persist g_queries to NVS key "queries"
int  settingsCountQueries();        // number of active (used != 0) queries

bool settingsIsValidHostname(const char* name);
bool settingsIsValidJoint(const JointCalibration& j);
bool settingsIsValidModel(const KinematicModel& m);
void settingsSanitizeCamera(CameraSettings& c);

bool settingsIsValidQueryName(const char* name);
bool settingsIsValidStoredQuery(const StoredQuery& q);
bool settingsParseQuery(const char* anglesStr, const char* speedsStr,
                        int16_t outTenths[NUM_JOINTS], uint8_t outSpeeds[NUM_JOINTS],
                        char* errBuf, size_t errLen);
void settingsFormatQueryAngles(const int16_t tenths[NUM_JOINTS], char* outBuf, size_t outLen);
void settingsFormatQuerySpeeds(const uint8_t speeds[NUM_JOINTS], char* outBuf, size_t outLen);

#endif // SETTINGS_H
