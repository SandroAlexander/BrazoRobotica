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

// Default Denavit-Hartenberg parameters (alpha deg, a mm, d mm, theta_0 deg)
// Matches the 6-DOF arm geometry (L0=70, L1=105, L2=100, L3=120) so that at
// kinematic zero (all q_i = 0) the arm points straight up at (0, 0, 395 mm).
const DHParam DEFAULT_DH_PARAMS[NUM_JOINTS] = {
  // alpha    a               d                theta
  { -90.0f,   0.0f,           DEFAULT_LINK_L0,   0.0f }, // J0 Base
  {   0.0f,   DEFAULT_LINK_L1, 0.0f,           -90.0f }, // J1 Shoulder
  {  90.0f,   0.0f,           0.0f,             90.0f }, // J2 Elbow
  { -90.0f,   0.0f,           DEFAULT_LINK_L2,   0.0f }, // J3 Wrist
  {  90.0f,   0.0f,           0.0f,              0.0f }, // J4 Hand
  {   0.0f,   0.0f,           DEFAULT_LINK_L3,   0.0f }  // J5 Gripper / Tool
};
