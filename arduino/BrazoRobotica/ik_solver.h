#ifndef IK_SOLVER_H
#define IK_SOLVER_H

// =====================================================================
//  Inverse Kinematics – Denavit-Hartenberg (4x4 Homogeneous Matrix) &
//  Differential Evolution Solver
//
//  Pure C++ (no Arduino dependencies) so it can also be unit-tested on a PC.
//
//  Joint angles exchanged with the solver are SERVO degrees; the mapping to
//  kinematic angles uses JointCalibration::kinZero and ::kinDir plus the
//  Denavit-Hartenberg theta offset:
//      theta_eff_deg = (servoDeg - kinZero) * kinDir + dh[i].theta
// =====================================================================

#include "config.h"

// 4x4 Homogeneous transformation matrix
struct Mat4 {
  float m[4][4];
};

// Kinematic model stored as 6 Denavit-Hartenberg rows (alpha, a, d, theta)
struct KinematicModel {
  DHParam dh[NUM_JOINTS];
};

struct IKTarget {
  float x, y, z;          // desired tool-tip position (mm)
  bool  useOrientation;   // also constrain the tool approach direction
  float pitchDeg;         // approach angle vs. horizontal (-90 = pointing down)
  int   iterations;       // Differential Evolution max iterations (0 = default)
  int   population;       // Differential Evolution population size P (0 = default)
  bool  lockGripper;      // keep J5 (Gripper) at current seed angle
};

struct IKResult {
  bool  success;
  int   iterations;              // iterations executed
  int   population;              // population size used
  float positionError;           // Euclidean 3D distance to target (mm)
  float orientationError;        // |a_desired - a| (0 when not used)
  float angles[NUM_JOINTS];      // solved servo degrees (always within limits)
};

// Build a single 4x4 Denavit-Hartenberg homogeneous matrix from (alpha, a, d, theta)
// with alpha and theta in degrees, a and d in millimetres.
Mat4 dhMatrix(float alphaDeg, float a, float d, float thetaDeg);

// Multiply two 4x4 homogeneous matrices: R = A * B
Mat4 mat4Mul(const Mat4& A, const Mat4& B);

// Forward kinematics using the 6-joint Denavit-Hartenberg chain.
// - pos[3]              : tool tip (x, y, z) in mm
// - approach[3]         : optional (may be nullptr) unit tool approach axis (Z_6)
// - relOrigins[6][3]    : optional (may be nullptr) 3D origin vector (dx, dy, dz)
//                         of each bone relative to the previous bone in the chain
void ikForward(const float servoDeg[NUM_JOINTS],
               const JointCalibration cal[NUM_JOINTS],
               const KinematicModel& model,
               float pos[3],
               float approach[3],
               float relOrigins[NUM_JOINTS][3] = nullptr);

// Core Differential Evolution solver:
//   solve(px, py, pz, iterations, population, ...) -> outAngles[NUM_JOINTS]
// Returns true if final position error <= IK_POSITION_TOLERANCE_MM.
bool solve(float px, float py, float pz,
           int iterations, int population,
           const float seed[NUM_JOINTS],
           const JointCalibration cal[NUM_JOINTS],
           const KinematicModel& model,
           float outAngles[NUM_JOINTS],
           IKResult* outDetail = nullptr,
           const IKTarget* fullTarget = nullptr);

// High-level IK entry point used by the robot controller.
IKResult ikSolve(const IKTarget& target,
                 const float seed[NUM_JOINTS],
                 const JointCalibration cal[NUM_JOINTS],
                 const KinematicModel& model);

#endif // IK_SOLVER_H
