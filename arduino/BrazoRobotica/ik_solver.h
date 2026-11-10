#ifndef IK_SOLVER_H
#define IK_SOLVER_H

// =====================================================================
//  Iterative inverse kinematics - damped least squares (Levenberg-Marquardt)
//
//  Pure C++ (no Arduino dependencies) so it can also be unit-tested on a PC.
//
//  Kinematic chain (base frame: Z up, X forward at base kinematic zero):
//    J0 Base     - yaw   about Z
//    J1 Shoulder - pitch (0 = link pointing straight up)
//    J2 Elbow    - pitch relative to upper arm (0 = straight)
//    J3 Wrist    - roll  about forearm axis   (kept fixed by the solver)
//    J4 Hand     - pitch relative to forearm  (0 = straight)
//    J5 Gripper  - not part of the kinematic chain
//
//  Joint angles exchanged with the solver are SERVO degrees; the mapping to
//  kinematic angles uses JointCalibration::kinZero and ::kinDir:
//      q_kin = (servoDeg - kinZero) * kinDir
// =====================================================================

#include "config.h"

struct KinematicModel {
  float L0;   // base -> shoulder height (mm)
  float L1;   // upper arm (mm)
  float L2;   // forearm (mm)
  float L3;   // hand + gripper (mm)
};

struct IKTarget {
  float x, y, z;          // tool tip position, mm
  bool  useOrientation;   // also constrain the tool approach direction
  float pitchDeg;         // approach angle vs. horizontal (-90 = pointing down)
};

struct IKResult {
  bool  success;
  int   iterations;              // total iterations over all attempts
  float positionError;           // mm
  float orientationError;        // |a_desired - a| (0 when not used)
  float angles[NUM_JOINTS];      // servo degrees (always within limits)
};

// Forward kinematics. pos: tool tip (mm). approach (optional, may be null): unit tool axis.
void ikForward(const float servoDeg[NUM_JOINTS],
               const JointCalibration cal[NUM_JOINTS],
               const KinematicModel& model,
               float pos[3], float approach[3]);

// Iterative IK. 'seed' is normally the current joint state (gives the
// closest solution and smooth motion). Joints 3 (wrist roll) and 5
// (gripper) are copied from the seed unchanged.
IKResult ikSolve(const IKTarget& target,
                 const float seed[NUM_JOINTS],
                 const JointCalibration cal[NUM_JOINTS],
                 const KinematicModel& model);

#endif // IK_SOLVER_H
