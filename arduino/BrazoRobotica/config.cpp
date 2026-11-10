// Default calibration values for each joint – adjust these to match your hardware.
// They can also be edited at runtime from the Web UI (Settings panel) and are then
// stored in NVS, overriding these factory values.
#include "config.h"

// Example values (minPulseUs, maxPulseUs correspond to minAngle and maxAngle)
const JointCalibration DEFAULT_JOINT_CALIBRATIONS[NUM_JOINTS] = {
  // ch  minUs  maxUs  minA    maxA    home   kinZero dir
  {  0,  500,  2500,   0.0f, 360.0f, 180.0f, 180.0f,  1 }, // Base     (Z axis, 360°)
  {  1,  500,  2500,   0.0f, 180.0f,  90.0f,  90.0f,  1 }, // Shoulder (pitch)
  {  2,  500,  2500,   0.0f, 180.0f,  90.0f,  90.0f,  1 }, // Elbow    (pitch)
  {  3,  500,  2500,   0.0f, 360.0f, 180.0f, 180.0f,  1 }, // Wrist    (roll, 360°)
  {  4,  500,  2500,   0.0f, 180.0f,  90.0f,  90.0f,  1 }, // Hand     (pitch)
  {  5,  500,  2500,   0.0f, 180.0f,  90.0f,  90.0f,  1 }  // Gripper  (open/close)
};

const char* const JOINT_NAMES[NUM_JOINTS] = {
  "Base", "Shoulder", "Elbow", "Wrist", "Hand", "Gripper"
};
