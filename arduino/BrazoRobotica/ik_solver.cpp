// =====================================================================
//  Inverse Kinematics – Denavit-Hartenberg (4x4 Homogeneous Matrix) &
//  Differential Evolution Solver
// =====================================================================
#include "ik_solver.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {

const float kDeg2Rad = 0.017453292519943295f;
const float kRad2Deg = 57.29577951308232f;

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float dirOf(const JointCalibration& c) {
  return c.kinDir < 0 ? -1.0f : 1.0f;
}

float servoToKinDeg(float servoDeg, const JointCalibration& c) {
  return (servoDeg - c.kinZero) * dirOf(c);
}

float kinDegToServo(float qDeg, const JointCalibration& c) {
  return c.kinZero + qDeg * dirOf(c);
}

// Fast deterministic PRNG (xorshift32)
struct Rng {
  uint32_t state;
  explicit Rng(uint32_t seed) : state(seed ? seed : 0x9E3779B9u) {}

  uint32_t nextU32() {
    uint32_t x = state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    state = x;
    return x;
  }

  // Uniform float in [0, 1)
  float uniform01() {
    return (nextU32() & 0x00FFFFFFu) * (1.0f / 16777216.0f);
  }

  // Uniform float in [lo, hi]
  float uniformRange(float lo, float hi) {
    return lo + uniform01() * (hi - lo);
  }

  // Uniform integer in [0, n - 1]
  int nextInt(int n) {
    return n > 0 ? (int)(nextU32() % (uint32_t)n) : 0;
  }
};

struct Eval {
  float dist;     // total objective distance (mm + weighted orientation)
  float posErr;   // Euclidean 3D position distance (mm)
  float oriErr;   // |a_desired - a|
};

Eval evaluateMember(const float servo[NUM_JOINTS],
                    const JointCalibration cal[NUM_JOINTS],
                    const KinematicModel& model,
                    const IKTarget& target,
                    const float aDes[3]) {
  float p[3], a[3];
  ikForward(servo, cal, model, p, target.useOrientation ? a : nullptr, nullptr);
  const float dx = target.x - p[0];
  const float dy = target.y - p[1];
  const float dz = target.z - p[2];

  Eval e;
  e.posErr = sqrtf(dx * dx + dy * dy + dz * dz);
  e.oriErr = 0.0f;
  e.dist   = e.posErr;

  if (target.useOrientation) {
    const float ox = aDes[0] - a[0];
    const float oy = aDes[1] - a[1];
    const float oz = aDes[2] - a[2];
    e.oriErr = sqrtf(ox * ox + oy * oy + oz * oz);
    const float w = IK_ORIENT_WEIGHT * e.oriErr;
    e.dist = sqrtf(e.posErr * e.posErr + w * w);
  }
  return e;
}

bool isConverged(const Eval& e, const IKTarget& t) {
  return e.posErr <= IK_POSITION_TOLERANCE_MM &&
         (!t.useOrientation || e.oriErr <= IK_ORIENT_TOLERANCE);
}

// ---------------------------------------------------------------------
//  Differential Evolution stages:
//    populate()  -> generate initial population of P 6-angle vectors
//    mutate()    -> create mutant vector Vi = Xbase + K * (Xj - Xk)
//    crossover() -> create candidate vector Ui from Xi and Vi
//    compare()   -> evaluate distance of Ui vs Xi and keep the closer one
// ---------------------------------------------------------------------

void populate(float (*pop)[NUM_JOINTS],
              Eval* costs,
              int P,
              const float seed[NUM_JOINTS],
              const JointCalibration cal[NUM_JOINTS],
              const KinematicModel& model,
              const IKTarget& target,
              const float aDes[3],
              float yawDeg,
              Rng& rng,
              int& bestIdx) {
  bestIdx = 0;
  for (int i = 0; i < P; ++i) {
    if (i == 0 && seed != nullptr) {
      // Member 0: current arm angles (ensures we never do worse than starting pose)
      for (int g = 0; g < NUM_JOINTS; ++g) {
        pop[i][g] = clampf(seed[g], cal[g].minAngle, cal[g].maxAngle);
      }
    } else {
      for (int g = 0; g < NUM_JOINTS; ++g) {
        pop[i][g] = rng.uniformRange(cal[g].minAngle, cal[g].maxAngle);
      }
      // Guide half of the initial population's base joint toward the target's azimuth
      // and wrist roll near neutral to accelerate convergence in 3D space.
      if (i % 2 == 1) {
        const float baseTarget = kinDegToServo(yawDeg, cal[0]);
        const float jitter = (i == 1) ? 0.0f : rng.uniformRange(-15.0f, 15.0f);
        pop[i][0] = clampf(baseTarget + jitter, cal[0].minAngle, cal[0].maxAngle);
        pop[i][3] = clampf(cal[3].kinZero, cal[3].minAngle, cal[3].maxAngle);
      }
    }
    if (target.lockGripper && seed != nullptr) {
      pop[i][5] = clampf(seed[5], cal[5].minAngle, cal[5].maxAngle);
    }

    costs[i] = evaluateMember(pop[i], cal, model, target, aDes);
    if (costs[i].dist < costs[bestIdx].dist) {
      bestIdx = i;
    }
  }
}

void mutate(const float (*pop)[NUM_JOINTS],
            int i,
            int P,
            int bestIdx,
            float K,
            const JointCalibration cal[NUM_JOINTS],
            bool lockGripper,
            float lockedGripperAngle,
            Rng& rng,
            float outMutant[NUM_JOINTS]) {
  // Pick random indices j and k different from i (and from each other)
  int j = rng.nextInt(P - 1);
  if (j >= i) ++j;

  int k = rng.nextInt(P - 2);
  if (k >= (i < j ? i : j)) ++k;
  if (k >= (i > j ? i : j)) ++k;

  const float* baseVec = ((i & 1) == 0) ? pop[bestIdx] : pop[i];

  for (int g = 0; g < NUM_JOINTS; ++g) {
    if (lockGripper && g == 5) {
      outMutant[g] = lockedGripperAngle;
      continue;
    }
    // Vi = Xbase + K * (Xj - Xk), clamped to valid joint limits
    const float v = baseVec[g] + K * (pop[j][g] - pop[k][g]);
    outMutant[g] = clampf(v, cal[g].minAngle, cal[g].maxAngle);
  }
}

void crossover(const float xi[NUM_JOINTS],
               const float vi[NUM_JOINTS],
               float CR,
               bool lockGripper,
               float lockedGripperAngle,
               Rng& rng,
               float outTrial[NUM_JOINTS]) {
  const int activeGenes = lockGripper ? (NUM_JOINTS - 1) : NUM_JOINTS;
  const int gForce = rng.nextInt(activeGenes);
  for (int g = 0; g < NUM_JOINTS; ++g) {
    if (lockGripper && g == 5) {
      outTrial[g] = lockedGripperAngle;
      continue;
    }
    // Ui = {Xig if rand(0, 1) < CR else Vig} (with forced mutant gene gForce)
    if (g != gForce && rng.uniform01() < CR) {
      outTrial[g] = xi[g];
    } else {
      outTrial[g] = vi[g];
    }
  }
}

void compare(float xi[NUM_JOINTS],
             Eval& costI,
             const float ui[NUM_JOINTS],
             int i,
             const JointCalibration cal[NUM_JOINTS],
             const KinematicModel& model,
             const IKTarget& target,
             const float aDes[3],
             Eval& bestCost,
             int& bestIdx) {
  const Eval trialEval = evaluateMember(ui, cal, model, target, aDes);
  if (trialEval.dist <= costI.dist) {
    memcpy(xi, ui, sizeof(float) * NUM_JOINTS);
    costI = trialEval;
    if (trialEval.dist < bestCost.dist) {
      bestCost = trialEval;
      bestIdx = i;
    }
  }
}

} // namespace

// ---------------------------------------------------------------------
Mat4 dhMatrix(float alphaDeg, float a, float d, float thetaDeg) {
  const float alpha = alphaDeg * kDeg2Rad;
  const float theta = thetaDeg * kDeg2Rad;
  const float ct = cosf(theta), st = sinf(theta);
  const float ca = cosf(alpha), sa = sinf(alpha);

  Mat4 T = {{
    { ct, -st * ca,  st * sa, a * ct },
    { st,  ct * ca, -ct * sa, a * st },
    { 0.0f,     sa,       ca,      d },
    { 0.0f,   0.0f,     0.0f,   1.0f }
  }};
  return T;
}

Mat4 mat4Mul(const Mat4& A, const Mat4& B) {
  Mat4 R;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      R.m[i][j] = A.m[i][0] * B.m[0][j] +
                  A.m[i][1] * B.m[1][j] +
                  A.m[i][2] * B.m[2][j] +
                  A.m[i][3] * B.m[3][j];
    }
  }
  return R;
}

void ikForward(const float servoDeg[NUM_JOINTS],
               const JointCalibration cal[NUM_JOINTS],
               const KinematicModel& model,
               float pos[3],
               float approach[3],
               float relOrigins[NUM_JOINTS][3]) {
  Mat4 T = {{
    { 1.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 1.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 1.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 1.0f }
  }};

  float prevOrigin[3] = { 0.0f, 0.0f, 0.0f };

  for (int i = 0; i < NUM_JOINTS; ++i) {
    const float qDeg = servoToKinDeg(servoDeg[i], cal[i]);
    const float effTheta = qDeg + model.dh[i].theta;
    const Mat4 Ai = dhMatrix(model.dh[i].alpha, model.dh[i].a, model.dh[i].d, effTheta);
    T = mat4Mul(T, Ai);

    if (relOrigins) {
      // 3D position vector of bone i's origin relative to bone (i-1)'s origin in base frame
      relOrigins[i][0] = T.m[0][3] - prevOrigin[0];
      relOrigins[i][1] = T.m[1][3] - prevOrigin[1];
      relOrigins[i][2] = T.m[2][3] - prevOrigin[2];
      prevOrigin[0] = T.m[0][3];
      prevOrigin[1] = T.m[1][3];
      prevOrigin[2] = T.m[2][3];
    }
  }

  if (pos) {
    pos[0] = T.m[0][3];
    pos[1] = T.m[1][3];
    pos[2] = T.m[2][3];
  }
  if (approach) {
    approach[0] = T.m[0][2];
    approach[1] = T.m[1][2];
    approach[2] = T.m[2][2];
  }
}

bool solve(float px, float py, float pz,
           int iterations, int population,
           const float seed[NUM_JOINTS],
           const JointCalibration cal[NUM_JOINTS],
           const KinematicModel& model,
           float outAngles[NUM_JOINTS],
           IKResult* outDetail,
           const IKTarget* fullTarget) {
  IKTarget target;
  if (fullTarget) {
    target = *fullTarget;
  } else {
    memset(&target, 0, sizeof(target));
    target.lockGripper = true;
  }
  target.x = px;
  target.y = py;
  target.z = pz;

  const int maxIter = (iterations > 0)
                        ? (iterations > IK_MAX_ITERATIONS_LIMIT ? IK_MAX_ITERATIONS_LIMIT : iterations)
                        : IK_DEFAULT_ITERATIONS;
  int P = (population > 0) ? population : IK_DEFAULT_POPULATION;
  if (P < 4) P = 4;
  if (P > IK_MAX_POPULATION_LIMIT) P = IK_MAX_POPULATION_LIMIT;

  float defaultSeed[NUM_JOINTS];
  if (!seed) {
    for (int i = 0; i < NUM_JOINTS; ++i) defaultSeed[i] = cal[i].homeAngle;
    seed = defaultSeed;
  }

  // Heading towards the target in the X-Y plane
  float yawRad = servoToKinDeg(seed[0], cal[0]) * kDeg2Rad;
  if (sqrtf(target.x * target.x + target.y * target.y) > 1.0f) {
    yawRad = atan2f(target.y, target.x);
  }
  const float yawDeg = yawRad * kRad2Deg;

  float aDes[3] = { 0.0f, 0.0f, 1.0f };
  if (target.useOrientation) {
    const float ph = target.pitchDeg * kDeg2Rad;
    aDes[0] = cosf(ph) * cosf(yawRad);
    aDes[1] = cosf(ph) * sinf(yawRad);
    aDes[2] = sinf(ph);
  }

  // Allocate only pop (P * 6 floats) and costs (P Evals) on the standard heap
  // (~1.4 KB for default P=40) to keep loopTask stack usage under 150 bytes
  // without touching PSRAM or permanent BSS memory.
  const size_t popBytes   = (size_t)P * sizeof(float) * NUM_JOINTS;
  const size_t costsBytes = (size_t)P * sizeof(Eval);
  uint8_t* rawMem = static_cast<uint8_t*>(malloc(popBytes + costsBytes));
  if (!rawMem) {
    memcpy(outAngles, seed, sizeof(float) * NUM_JOINTS);
    const Eval fallbackEval = evaluateMember(outAngles, cal, model, target, aDes);
    const bool converged = isConverged(fallbackEval, target);
    if (outDetail) {
      outDetail->success          = converged;
      outDetail->iterations       = 0;
      outDetail->population       = 0;
      outDetail->positionError    = fallbackEval.posErr;
      outDetail->orientationError = target.useOrientation ? fallbackEval.oriErr : 0.0f;
      memcpy(outDetail->angles, outAngles, sizeof(outDetail->angles));
    }
    return converged;
  }

  float (*pop)[NUM_JOINTS] = reinterpret_cast<float (*)[NUM_JOINTS]>(rawMem);
  Eval* costs              = reinterpret_cast<Eval*>(rawMem + popBytes);

  const float lockedGripper = clampf(seed[5], cal[5].minAngle, cal[5].maxAngle);
  const uint32_t rngSeed = (uint32_t)(fabsf(px * 131.0f) + fabsf(py * 257.0f) + fabsf(pz * 521.0f)) ^ 0xA53C9E4Du;
  Rng rng(rngSeed);

  int bestIdx = 0;
  populate(pop, costs, P, seed, cal, model, target, aDes, yawDeg, rng, bestIdx);

  const float keepXProb = 1.0f - IK_DE_CROSSOVER_CR;
  float mutant[NUM_JOINTS];
  float trial[NUM_JOINTS];

  int iterDone = 0;
  for (int it = 0; it < maxIter; ++it) {
    if (isConverged(costs[bestIdx], target)) break;
    ++iterDone;

    for (int i = 0; i < P; ++i) {
      mutate(pop, i, P, bestIdx, IK_DE_MUTATION_K, cal, target.lockGripper, lockedGripper, rng, mutant);
      crossover(pop[i], mutant, keepXProb, target.lockGripper, lockedGripper, rng, trial);
      compare(pop[i], costs[i], trial, i, cal, model, target, aDes, costs[bestIdx], bestIdx);
    }
  }

  memcpy(outAngles, pop[bestIdx], sizeof(float) * NUM_JOINTS);
  const Eval bestEval = costs[bestIdx];
  free(rawMem);

  const bool converged = isConverged(bestEval, target);

  if (outDetail) {
    outDetail->success          = converged;
    outDetail->iterations       = iterDone;
    outDetail->population       = P;
    outDetail->positionError    = bestEval.posErr;
    outDetail->orientationError = target.useOrientation ? bestEval.oriErr : 0.0f;
    memcpy(outDetail->angles, outAngles, sizeof(outDetail->angles));
  }
  return converged;
}

IKResult ikSolve(const IKTarget& target,
                 const float seed[NUM_JOINTS],
                 const JointCalibration cal[NUM_JOINTS],
                 const KinematicModel& model) {
  IKResult res;
  memset(&res, 0, sizeof(res));
  float angles[NUM_JOINTS];
  solve(target.x, target.y, target.z,
        target.iterations, target.population,
        seed, cal, model, angles, &res, &target);
  return res;
}
