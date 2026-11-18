#include "pwm_driver.h"
#include "settings.h"
#include "emergency_stop.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

namespace {
Adafruit_PWMServoDriver s_pca(PCA9685_I2C_ADDRESS, Wire);
bool     s_online = false;
float    s_current[NUM_JOINTS];
float    s_target[NUM_JOINTS];
uint8_t  s_jointSpeedDps[NUM_JOINTS];
bool     s_usePerJointSpeeds = false;
uint16_t s_lastPulse[NUM_JOINTS];
uint32_t s_lastUpdateMs = 0;
uint32_t s_lastProbeMs = 0;

void recoverI2CBus() {
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  delay(10);

  // If SDA is held low by a slave device waiting for clock pulses, clock SCL up to 9 times
  if (digitalRead(I2C_SDA_PIN) == LOW) {
    Serial.println("[I2C] SDA held LOW at startup – performing 9-pulse bus recovery...");
    pinMode(I2C_SCL_PIN, OUTPUT);
    for (int i = 0; i < 9; ++i) {
      digitalWrite(I2C_SCL_PIN, LOW);
      delayMicroseconds(10);
      digitalWrite(I2C_SCL_PIN, HIGH);
      delayMicroseconds(10);
      if (digitalRead(I2C_SDA_PIN) == HIGH) break;
    }
    // Generate an I2C STOP condition
    pinMode(I2C_SDA_PIN, OUTPUT);
    digitalWrite(I2C_SDA_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(I2C_SDA_PIN, HIGH);
    delayMicroseconds(10);
  }
}

void scanI2CBus() {
  Serial.println("[I2C] Scanning I2C bus for devices (0x01..0x7F)...");
  uint8_t count = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    const uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("[I2C] Device found at 0x%02X", addr);
      if (addr == PCA9685_I2C_ADDRESS) {
        Serial.println(" (PCA9685 Default)");
      } else if (addr >= 0x40 && addr <= 0x47) {
        Serial.println(" (PCA9685 Alternate address)");
      } else if (addr == 0x70) {
        Serial.println(" (PCA9685 All-Call address)");
      } else {
        Serial.println();
      }
      count++;
    }
  }
  if (count == 0) {
    Serial.println("[I2C] ERROR: No I2C devices responded!");
    Serial.println("      1. Check SDA (GPIO 14) and SCL (GPIO 15) wire assignments.");
    Serial.println("      2. Verify PCA9685 pin order: GND, OE, SCL, SDA, VCC, V+.");
    Serial.println("      3. Verify PCA9685 VCC is firmly connected to ESP32 3.3V.");
    Serial.println("      4. Verify PCA9685 GND is connected to ESP32 GND.");
  } else {
    Serial.printf("[I2C] Scan complete: %d device(s) found.\n", count);
  }
}

bool probe() {
  Wire.beginTransmission(PCA9685_I2C_ADDRESS);
  return Wire.endTransmission() == 0;
}

void configureChip() {
  s_pca.begin();
  s_pca.setOscillatorFrequency(27000000);
  s_pca.setPWMFreq(PWM_FREQUENCY);
  delay(10);
}

float clampToJoint(uint8_t id, float angle) {
  const JointCalibration& c = g_settings.joints[id];
  if (isnan(angle)) return s_target[id];
  return constrain(angle, c.minAngle, c.maxAngle);
}

void writeJoint(uint8_t id, bool force) {
  const uint16_t us = angleToPulseUs(g_settings.joints[id], s_current[id]);
  if (!force && us == s_lastPulse[id]) return;
  s_lastPulse[id] = us;
  if (!s_online) return;

#if USE_PWM_PHASE_SHIFT
  // Convert pulse in microseconds to 12-bit PCA9685 tick count (0..4095) at 50 Hz (20,000 us)
  const uint16_t pulseTicks = (uint16_t)(((uint32_t)us * 4096ULL + 10000ULL) / 20000ULL);
  // Stagger the turn-on tick by 256 counts (~1.25 ms) per channel to phase-shift current draw
  const uint16_t onTick = (uint16_t)((id * 256) % 4096);
  const uint16_t offTick = (uint16_t)((onTick + pulseTicks) % 4096);
  s_pca.setPWM(g_settings.joints[id].channel, onTick, offTick);
#else
  s_pca.writeMicroseconds(g_settings.joints[id].channel, us);
#endif
}

void writeAll(bool force) {
  for (uint8_t i = 0; i < NUM_JOINTS; ++i) writeJoint(i, force);
}

void writeAllStaggered(bool force) {
  for (uint8_t i = 0; i < NUM_JOINTS; ++i) {
    writeJoint(i, force);
    delay(SOFT_START_DELAY_MS);
  }
}
} // namespace

uint16_t angleToPulseUs(const JointCalibration& c, float angle) {
  const float a = constrain(angle, c.minAngle, c.maxAngle);
  const float span = c.maxAngle - c.minAngle;
  const float t = span > 0.0f ? (a - c.minAngle) / span : 0.0f;
  return (uint16_t)lroundf(c.minPulseUs + t * (float)(c.maxPulseUs - c.minPulseUs));
}

bool initPWM() {
  recoverI2CBus();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);
  Wire.setTimeOut(25); // Prevent lockup if SDA/SCL lines are floating or disconnected
  for (uint8_t i = 0; i < NUM_JOINTS; ++i) {
    s_current[i] = s_target[i] = g_settings.joints[i].homeAngle;
    s_jointSpeedDps[i] = 0;
    s_lastPulse[i] = 0;
  }
  s_usePerJointSpeeds = false;
  s_online = probe();
  if (s_online) {
    configureChip();
    writeAllStaggered(true);
    Serial.println("[PWM] PCA9685 online, joints moved to home (staggered soft-start)");
  } else {
    Serial.println("[PWM] PCA9685 NOT found at address 0x40 - running I2C bus scan...");
    scanI2CBus();
    Serial.println("[PWM] Check wiring / power (will keep retrying every 2s)");
  }
  s_lastUpdateMs = millis();
  return s_online;
}

bool pwmIsOnline() { return s_online; }

void clearPerJointSpeeds() {
  for (uint8_t i = 0; i < NUM_JOINTS; ++i) s_jointSpeedDps[i] = 0;
  s_usePerJointSpeeds = false;
}

void setJointAngle(uint8_t id, float angle) {
  if (id >= NUM_JOINTS) return;
  clearPerJointSpeeds();
  s_target[id] = clampToJoint(id, angle);
}

void setAllJointAngles(const float angles[NUM_JOINTS]) {
  clearPerJointSpeeds();
  for (uint8_t i = 0; i < NUM_JOINTS; ++i) s_target[i] = clampToJoint(i, angles[i]);
}

void setJointAngleWithSpeed(uint8_t id, float angle, uint8_t speedDps) {
  if (id >= NUM_JOINTS) return;
  s_usePerJointSpeeds = true;
  s_jointSpeedDps[id] = speedDps;
  s_target[id] = clampToJoint(id, angle);
}

void setAllJointAnglesWithSpeeds(const float angles[NUM_JOINTS], const uint8_t speedsDps[NUM_JOINTS]) {
  s_usePerJointSpeeds = true;
  for (uint8_t i = 0; i < NUM_JOINTS; ++i) {
    s_target[i] = clampToJoint(i, angles[i]);
    s_jointSpeedDps[i] = speedsDps ? speedsDps[i] : 0;
  }
}

void moveHome() {
  clearPerJointSpeeds();
  for (uint8_t i = 0; i < NUM_JOINTS; ++i) s_target[i] = g_settings.joints[i].homeAngle;
}

void holdPosition() {
  clearPerJointSpeeds();
  for (uint8_t i = 0; i < NUM_JOINTS; ++i) s_target[i] = s_current[i];
}

float getJointAngle(uint8_t id) { return id < NUM_JOINTS ? s_current[id] : NAN; }

void getJointAngles(float out[NUM_JOINTS]) {
  for (uint8_t i = 0; i < NUM_JOINTS; ++i) out[i] = s_current[i];
}

void getTargetAngles(float out[NUM_JOINTS]) {
  for (uint8_t i = 0; i < NUM_JOINTS; ++i) out[i] = s_target[i];
}

bool isMoving() {
  for (uint8_t i = 0; i < NUM_JOINTS; ++i)
    if (fabsf(s_target[i] - s_current[i]) > 0.01f) return true;
  return false;
}

void reclampTargets() {
  for (uint8_t i = 0; i < NUM_JOINTS; ++i) {
    const JointCalibration& c = g_settings.joints[i];
    s_target[i]  = constrain(s_target[i],  c.minAngle, c.maxAngle);
    s_current[i] = constrain(s_current[i], c.minAngle, c.maxAngle);
  }
  writeAll(true);
}

void motionUpdate() {
  const uint32_t now = millis();

  // Hot-plug: retry PCA9685 detection every 2 s while offline
  if (!s_online && now - s_lastProbeMs > 2000) {
    s_lastProbeMs = now;
    if (probe()) {
      s_online = true;
      configureChip();
      writeAllStaggered(true);
      Serial.println("[PWM] PCA9685 detected (staggered soft-start)");
    }
  }

  const uint32_t dtMs = now - s_lastUpdateMs;
  if (dtMs < MOTION_UPDATE_PERIOD_MS) return;
  s_lastUpdateMs = now;

  // Emergency stop: freeze where we are
  if (estopIsActive()) {
    holdPosition();
    return;
  }

  // Check if any joint needs to move
  float maxDelta = 0.0f;
  for (uint8_t i = 0; i < NUM_JOINTS; ++i) maxDelta = fmaxf(maxDelta, fabsf(s_target[i] - s_current[i]));
  if (maxDelta < 0.01f) {
    s_usePerJointSpeeds = false;
    return;
  }

  const float dtSec = (float)min<uint32_t>(dtMs, 100) / 1000.0f;

  if (s_usePerJointSpeeds) {
    // Per-articulation speed mode (used by movement queries): each joint moves at its own uint8_t speed (or global default if 0)
    for (uint8_t i = 0; i < NUM_JOINTS; ++i) {
      const float delta = s_target[i] - s_current[i];
      const float absDelta = fabsf(delta);
      if (absDelta < 0.01f) {
        s_current[i] = s_target[i];
        continue;
      }
      const float spd = s_jointSpeedDps[i] > 0 ? (float)s_jointSpeedDps[i] : g_settings.maxSpeedDps;
      const float step = spd * dtSec;
      if (absDelta <= step) {
        s_current[i] = s_target[i];
      } else {
        s_current[i] += (delta > 0.0f ? step : -step);
      }
    }
  } else {
    // Synchronized interpolation: every joint arrives at the same time
    const float maxStep = g_settings.maxSpeedDps * dtSec;
    const float ratio = maxDelta <= maxStep ? 1.0f : maxStep / maxDelta;
    for (uint8_t i = 0; i < NUM_JOINTS; ++i) {
      s_current[i] += (s_target[i] - s_current[i]) * ratio;
      if (ratio >= 1.0f) s_current[i] = s_target[i];
    }
  }
  writeAll(false);
}
