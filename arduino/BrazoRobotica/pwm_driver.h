#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

// =====================================================================
//  PCA9685 servo driver + motion engine
//  setJointAngle()/setAllJointAngles() only set TARGETS; motionUpdate()
//  (called from loop) interpolates all joints in a synchronized,
//  speed-limited straight line in joint space and writes the PCA9685.
//  All functions must be called from the main loop task.
// =====================================================================

#include "config.h"

bool  initPWM();                                  // I2C + PCA9685, moves to home immediately
bool  pwmIsOnline();                              // PCA9685 answered on the I2C bus

void  setJointAngle(uint8_t jointId, float angle);    // clamped to calibration limits
void  setAllJointAngles(const float angles[NUM_JOINTS]);
void  moveHome();
void  holdPosition();                             // target = current position (E-STOP)

float getJointAngle(uint8_t jointId);             // current (interpolated) angle
void  getJointAngles(float out[NUM_JOINTS]);
void  getTargetAngles(float out[NUM_JOINTS]);
bool  isMoving();

void  reclampTargets();                           // call after calibration changes
void  motionUpdate();                             // call every loop iteration

// Exposed for unit tests: angle (deg) -> pulse width (us) using a calibration
uint16_t angleToPulseUs(const JointCalibration& cal, float angle);

#endif // PWM_DRIVER_H
