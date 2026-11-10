// =====================================================================
//  Iterative inverse kinematics - damped least squares (Levenberg-Marquardt)
//  See ik_solver.h for the kinematic conventions.
// =====================================================================
#include "ik_solver.h"
#include <math.h>
#include <string.h>

namespace {

const float kDeg2Rad = 0.017453292519943295f;
const float kRad2Deg = 57.29577951308232f;

// Joints moved by the solver (wrist roll and gripper are kept fixed)
const int kNumActive = 4;
const int kActive[kNumActive] = {0, 1, 2, 4};

const float kLambdaMin = 0.01f;
const float kLambdaMax = 1.0e4f;

struct Mat3 { float m[3][3]; };

Mat3 mul(const Mat3& a, const Mat3& b) {
  Mat3 r;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
  return r;
}

Mat3 rotZ(float t) {
  const float c = cosf(t), s = sinf(t);
  Mat3 r = {{{c, -s, 0.0f}, {s, c, 0.0f}, {0.0f, 0.0f, 1.0f}}};
  return r;
}

Mat3 rotY(float t) {
  const float c = cosf(t), s = sinf(t);
  Mat3 r = {{{c, 0.0f, s}, {0.0f, 1.0f, 0.0f}, {-s, 0.0f, c}}};
  return r;
}

// p += R * (0, 0, len)  -> advance along the local link axis
void advance(const Mat3& R, float len, float p[3]) {
  p[0] += R.m[0][2] * len;
  p[1] += R.m[1][2] * len;
  p[2] += R.m[2][2] * len;
}

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

float dirOf(const JointCalibration& c) { return c.kinDir < 0 ? -1.0f : 1.0f; }

float servoToKin(float servo, const JointCalibration& c) {
  return (servo - c.kinZero) * dirOf(c) * kDeg2Rad;
}

float kinToServo(float qRad, const JointCalibration& c) {
  return c.kinZero + qRad * kRad2Deg * dirOf(c);
}

void fkFromKin(const float q[NUM_JOINTS], const KinematicModel& m, float pos[3], float a[3]) {
  Mat3 R = rotZ(q[0]);
  pos[0] = 0.0f; pos[1] = 0.0f; pos[2] = m.L0;
  R = mul(R, rotY(q[1])); advance(R, m.L1, pos);
  R = mul(R, rotY(q[2])); advance(R, m.L2, pos);
  R = mul(R, rotZ(q[3]));
  R = mul(R, rotY(q[4])); advance(R, m.L3, pos);
  if (a) { a[0] = R.m[0][2]; a[1] = R.m[1][2]; a[2] = R.m[2][2]; }
}

void toKin(const float servo[NUM_JOINTS], const JointCalibration cal[NUM_JOINTS], float q[NUM_JOINTS]) {
  for (int i = 0; i < NUM_JOINTS; ++i) q[i] = servoToKin(servo[i], cal[i]);
}

struct Eval { float cost, posErr, oriErr; };

Eval evaluate(const float servo[NUM_JOINTS], const JointCalibration cal[NUM_JOINTS],
              const KinematicModel& m, const IKTarget& t, const float aDes[3]) {
  float q[NUM_JOINTS], p[3], a[3];
  toKin(servo, cal, q);
  fkFromKin(q, m, p, a);
  const float dx = t.x - p[0], dy = t.y - p[1], dz = t.z - p[2];
  Eval e;
  e.posErr = sqrtf(dx * dx + dy * dy + dz * dz);
  e.oriErr = 0.0f;
  e.cost = e.posErr * e.posErr;
  if (t.useOrientation) {
    const float ox = aDes[0] - a[0], oy = aDes[1] - a[1], oz = aDes[2] - a[2];
    e.oriErr = sqrtf(ox * ox + oy * oy + oz * oz);
    const float w = IK_ORIENT_WEIGHT * e.oriErr;
    e.cost += w * w;
  }
  return e;
}

bool converged(const Eval& e, const IKTarget& t) {
  return e.posErr < IK_POSITION_TOLERANCE_MM && (!t.useOrientation || e.oriErr < IK_ORIENT_TOLERANCE);
}

// Solves A x = b (n x n) with partial pivoting. A and b are destroyed.
bool solveLinear(float A[kNumActive][kNumActive], float b[kNumActive], float x[kNumActive]) {
  const int n = kNumActive;
  for (int col = 0; col < n; ++col) {
    int piv = col;
    for (int r = col + 1; r < n; ++r)
      if (fabsf(A[r][col]) > fabsf(A[piv][col])) piv = r;
    if (fabsf(A[piv][col]) < 1e-9f) return false;
    if (piv != col) {
      for (int k = 0; k < n; ++k) { float tmp = A[col][k]; A[col][k] = A[piv][k]; A[piv][k] = tmp; }
      float tb = b[col]; b[col] = b[piv]; b[piv] = tb;
    }
    for (int r = col + 1; r < n; ++r) {
      const float f = A[r][col] / A[col][col];
      for (int k = col; k < n; ++k) A[r][k] -= f * A[col][k];
      b[r] -= f * b[col];
    }
  }
  for (int r = n - 1; r >= 0; --r) {
    float s = b[r];
    for (int k = r + 1; k < n; ++k) s -= A[r][k] * x[k];
    x[r] = s / A[r][r];
  }
  return true;
}

// One Levenberg-Marquardt run starting from 'servo' (modified in place).
Eval runAttempt(const IKTarget& t, const float aDes[3], float servo[NUM_JOINTS],
                const JointCalibration cal[NUM_JOINTS], const KinematicModel& m, int& iterations) {
  float lambda = IK_DAMPING;
  Eval cur = evaluate(servo, cal, m, t, aDes);

  for (int it = 0; it < IK_MAX_ITERATIONS; ++it) {
    if (converged(cur, t)) break;
    ++iterations;

    // --- current task vector -------------------------------------------
    float q[NUM_JOINTS], p[3], a[3];
    toKin(servo, cal, q);
    fkFromKin(q, m, p, a);

    const int rows = t.useOrientation ? 6 : 3;
    float e[6];
    e[0] = t.x - p[0]; e[1] = t.y - p[1]; e[2] = t.z - p[2];
    if (t.useOrientation) {
      e[3] = IK_ORIENT_WEIGHT * (aDes[0] - a[0]);
      e[4] = IK_ORIENT_WEIGHT * (aDes[1] - a[1]);
      e[5] = IK_ORIENT_WEIGHT * (aDes[2] - a[2]);
    }

    // --- numerical Jacobian d(task)/dq  (rows x 4) ---------------------
    float J[6][kNumActive];
    const float h = 1.0e-3f;
    for (int k = 0; k < kNumActive; ++k) {
      float qp[NUM_JOINTS];
      memcpy(qp, q, sizeof(qp));
      qp[kActive[k]] += h;
      float pp[3], ap[3];
      fkFromKin(qp, m, pp, ap);
      J[0][k] = (pp[0] - p[0]) / h;
      J[1][k] = (pp[1] - p[1]) / h;
      J[2][k] = (pp[2] - p[2]) / h;
      if (t.useOrientation) {
        J[3][k] = IK_ORIENT_WEIGHT * (ap[0] - a[0]) / h;
        J[4][k] = IK_ORIENT_WEIGHT * (ap[1] - a[1]) / h;
        J[5][k] = IK_ORIENT_WEIGHT * (ap[2] - a[2]) / h;
      }
    }

    // --- normal equations  (J^T J + lambda^2 I) dq = J^T e -------------
    float JtJ[kNumActive][kNumActive], Jte[kNumActive];
    for (int i = 0; i < kNumActive; ++i) {
      Jte[i] = 0.0f;
      for (int r = 0; r < rows; ++r) Jte[i] += J[r][i] * e[r];
      for (int j = 0; j < kNumActive; ++j) {
        float s = 0.0f;
        for (int r = 0; r < rows; ++r) s += J[r][i] * J[r][j];
        JtJ[i][j] = s;
      }
    }

    // Inner loop: increase damping until the step reduces the cost
    bool improved = false;
    while (lambda <= kLambdaMax) {
      float A[kNumActive][kNumActive], b[kNumActive], dq[kNumActive];
      for (int i = 0; i < kNumActive; ++i) {
        b[i] = Jte[i];
        for (int j = 0; j < kNumActive; ++j) A[i][j] = JtJ[i][j] + (i == j ? lambda * lambda : 0.0f);
      }
      if (!solveLinear(A, b, dq)) { lambda *= 4.0f; continue; }

      // limit step size
      float maxDeg = 0.0f;
      for (int k = 0; k < kNumActive; ++k) maxDeg = fmaxf(maxDeg, fabsf(dq[k]) * kRad2Deg);
      const float scale = maxDeg > IK_MAX_STEP_DEG ? IK_MAX_STEP_DEG / maxDeg : 1.0f;

      float cand[NUM_JOINTS];
      memcpy(cand, servo, sizeof(cand));
      for (int k = 0; k < kNumActive; ++k) {
        const int j = kActive[k];
        const float newQ = q[j] + dq[k] * scale;
        cand[j] = clampf(kinToServo(newQ, cal[j]), cal[j].minAngle, cal[j].maxAngle);
      }

      const Eval ce = evaluate(cand, cal, m, t, aDes);
      if (ce.cost < cur.cost) {
        memcpy(servo, cand, sizeof(cand));
        cur = ce;
        lambda = fmaxf(lambda * 0.5f, kLambdaMin);
        improved = true;
        break;
      }
      lambda *= 4.0f;
    }
    if (!improved) break;   // local minimum or joint limit reached
  }
  return cur;
}

} // namespace

// ---------------------------------------------------------------------
void ikForward(const float servoDeg[NUM_JOINTS], const JointCalibration cal[NUM_JOINTS],
               const KinematicModel& model, float pos[3], float approach[3]) {
  float q[NUM_JOINTS];
  toKin(servoDeg, cal, q);
  fkFromKin(q, model, pos, approach);
}

IKResult ikSolve(const IKTarget& target, const float seed[NUM_JOINTS],
                 const JointCalibration cal[NUM_JOINTS], const KinematicModel& model) {
  IKResult res;
  memset(&res, 0, sizeof(res));

  // Heading towards the target (keeps the current heading if the target is on the Z axis)
  float yaw = servoToKin(seed[0], cal[0]);
  if (sqrtf(target.x * target.x + target.y * target.y) > 1.0f) yaw = atan2f(target.y, target.x);

  float aDes[3] = {0.0f, 0.0f, 1.0f};
  if (target.useOrientation) {
    const float ph = target.pitchDeg * kDeg2Rad;
    aDes[0] = cosf(ph) * cosf(yaw);
    aDes[1] = cosf(ph) * sinf(yaw);
    aDes[2] = sinf(ph);
  }

  // Seeds: current pose, then a few canonical "elbow-up" poses facing the target
  const int kNumSeeds = 4;
  const float presets[kNumSeeds - 1][3] = {   // shoulder, elbow, hand (kinematic degrees)
    {  0.0f,  0.0f,  0.0f },
    { 30.0f, 60.0f, 60.0f },
    { 45.0f, 90.0f, 30.0f },
  };

  float bestAngles[NUM_JOINTS];
  Eval best = {1e30f, 1e30f, 1e30f};
  int iterations = 0;

  for (int s = 0; s < kNumSeeds; ++s) {
    float servo[NUM_JOINTS];
    memcpy(servo, seed, sizeof(servo));
    if (s > 0) {
      servo[0] = kinToServo(yaw, cal[0]);
      servo[1] = kinToServo(presets[s - 1][0] * kDeg2Rad, cal[1]);
      servo[2] = kinToServo(presets[s - 1][1] * kDeg2Rad, cal[2]);
      servo[4] = kinToServo(presets[s - 1][2] * kDeg2Rad, cal[4]);
    }
    for (int i = 0; i < NUM_JOINTS; ++i) servo[i] = clampf(servo[i], cal[i].minAngle, cal[i].maxAngle);

    const Eval e = runAttempt(target, aDes, servo, cal, model, iterations);
    if (e.cost < best.cost) {
      best = e;
      memcpy(bestAngles, servo, sizeof(bestAngles));
    }
    if (converged(e, target)) break;
  }

  res.success = converged(best, target);
  res.iterations = iterations;
  res.positionError = best.posErr;
  res.orientationError = target.useOrientation ? best.oriErr : 0.0f;
  memcpy(res.angles, bestAngles, sizeof(res.angles));
  return res;
}
