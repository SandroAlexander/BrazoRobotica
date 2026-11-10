#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "config.h"
#include "settings.h"
#include "pwm_driver.h"
#include "camera_interface.h"
#include "wifi_manager.h"
#include "robot_controller.h"

// ESP32-CAM onboard red status LED (GPIO 33, active LOW)
#define STATUS_LED_PIN 33

void setup() {
  // Disable brownout detector to prevent reset loops from USB voltage drops
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Initialize status LED (Blink twice to indicate boot)
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);  // ON
  delay(150);
  digitalWrite(STATUS_LED_PIN, HIGH); // OFF
  delay(150);
  digitalWrite(STATUS_LED_PIN, LOW);  // ON
  delay(150);
  digitalWrite(STATUS_LED_PIN, HIGH); // OFF

  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("==================================================");
  Serial.println("         BrazoRobotica - Arm Controller           ");
  Serial.println("==================================================");

  // 1. Load runtime configuration from NVS (or factory defaults)
  settingsBegin();

  // 2. Initialize Wi-Fi SoftAP FIRST so the network is always discoverable immediately
  Serial.println("[MAIN] Starting Wi-Fi AP & Web Servers...");
  if (wifiBegin()) {
    digitalWrite(STATUS_LED_PIN, LOW); // Solid ON = Wi-Fi AP active & healthy
  } else {
    Serial.println("[MAIN] ERROR: Wi-Fi initialization failed!");
  }

  // 3. Initialize PCA9685 servo driver & move joints to home position
  Serial.println("[MAIN] Initializing PWM Driver...");
  if (!initPWM()) {
    Serial.println("[MAIN] WARNING: PWM driver offline. Check I2C wiring/power.");
  }

  // 4. Initialize OV2640 camera & start MJPEG streaming server
  Serial.println("[MAIN] Initializing Camera Interface...");
  if (cameraBegin()) {
    cameraStartStreamServer();
  } else {
    Serial.println("[MAIN] WARNING: Camera initialization failed or not fitted.");
  }

  // 5. Initialize robot controller logic
  Serial.println("[MAIN] Starting Robot Controller...");
  controllerBegin();

  Serial.println("==================================================");
  Serial.printf(" System Ready! Access UI at: %s\n", wifiUrl().c_str());
  Serial.println("==================================================");
}

void loop() {
  // Service DNS queries and WebSocket client cleanup
  wifiLoop();

  // Process queued commands, perform kinematic interpolation, push telemetry
  controllerLoop();
}
