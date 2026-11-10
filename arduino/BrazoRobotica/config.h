#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// =====================================================================
//  BrazoRobotica – factory defaults
//  Values marked [NVS] can be changed from the Web UI and are persisted
//  in the ESP32 non-volatile storage. The values here are only used the
//  first time the board boots or after a "factory reset".
// =====================================================================

// ---------------------------------------------------------------------
//  Network
// ---------------------------------------------------------------------
#define DEFAULT_AP_SSID      "BrazoRobotica_AP"
#define DEFAULT_AP_PASSWORD  "robotarmpass"      // min. 8 characters
#define DEFAULT_HOSTNAME     "robot.arm.esp32"   // [NVS] UI address: http://robot.arm.esp32/
#define AP_CHANNEL           6
#define AP_MAX_CLIENTS       4
#define HTTP_PORT            80                  // UI, REST API and WebSocket (/ws)
#define STREAM_PORT          81                  // MJPEG stream (/stream)

// ---------------------------------------------------------------------
//  I2C bus for the PCA9685 servo driver
//  NOTE: GPIO 12 is a boot strapping pin (flash voltage select). The
//  pull-up resistors on most PCA9685 breakout boards hold it HIGH at
//  reset and the ESP32-CAM then fails to boot. GPIO 14/15 are safe.
//  Using 100 kHz standard mode ensures signal integrity over jumper wires
//  and avoids conflicts with the ESP32-CAM onboard MicroSD trace capacitance.
// ---------------------------------------------------------------------
#define I2C_SDA_PIN          14
#define I2C_SCL_PIN          15
#define I2C_CLOCK_HZ         100000
#define PCA9685_I2C_ADDRESS  0x40
#define PWM_FREQUENCY        50                  // Hz, standard hobby servos
#define SOFT_START_DELAY_MS  70                  // ms delay between joint enables during homing
#define USE_PWM_PHASE_SHIFT  true                // Stagger PWM edges to prevent simultaneous current draw

// ---------------------------------------------------------------------
//  Motion engine
// ---------------------------------------------------------------------
#define NUM_JOINTS               6
#define MOTION_UPDATE_PERIOD_MS  20      // 50 Hz interpolation loop
#define DEFAULT_MAX_SPEED_DPS    90.0f   // max. joint speed, degrees / second
#define TELEMETRY_PERIOD_MS      200     // WebSocket status broadcast period

// ---------------------------------------------------------------------
//  Safety
// ---------------------------------------------------------------------
#define EMERGENCY_LOCKOUT_MS     3000    // movement commands rejected for this long after E-STOP

// ---------------------------------------------------------------------
//  Kinematic model (millimetres)  [NVS]
//    L0: table -> shoulder axis height
//    L1: shoulder axis -> elbow axis
//    L2: elbow axis -> hand (pitch) axis
//    L3: hand axis -> gripper tip
// ---------------------------------------------------------------------
#define DEFAULT_LINK_L0          70.0f
#define DEFAULT_LINK_L1          105.0f
#define DEFAULT_LINK_L2          100.0f
#define DEFAULT_LINK_L3          120.0f

// Iterative IK solver (damped least squares)
#define IK_MAX_ITERATIONS        150
#define IK_POSITION_TOLERANCE_MM 1.0f
#define IK_ORIENT_TOLERANCE      0.02f   // |approach vector error| (unitless)
#define IK_DAMPING               8.0f    // lambda, larger = more stable / slower
#define IK_ORIENT_WEIGHT         60.0f   // mm-equivalent weight of the orientation error
#define IK_MAX_STEP_DEG          8.0f    // max. joint change per iteration

// ---------------------------------------------------------------------
//  Camera (AI-Thinker ESP32-CAM, OV2640)  [NVS]
// ---------------------------------------------------------------------
#define CAM_LED_PIN              4       // on-board flash LED
#define DEFAULT_CAM_FRAMESIZE    8       // FRAMESIZE_VGA (640x480)
#define DEFAULT_CAM_QUALITY      12      // JPEG quality 4..63 (lower = better)
#define DEFAULT_CAM_BRIGHTNESS   0       // -2..2
#define DEFAULT_CAM_CONTRAST     0       // -2..2
#define DEFAULT_CAM_SATURATION   0       // -2..2
#define DEFAULT_CAM_HMIRROR      0
#define DEFAULT_CAM_VFLIP        0
#define DEFAULT_CAM_LED_STATE    0       // LED disabled by default

// ---------------------------------------------------------------------
//  UI palette (used by web_ui.h – kept here for reference / theming)
// ---------------------------------------------------------------------
#define UI_COLOR_PRIMARY "#DAA520"   // gold
#define UI_COLOR_BG      "#212121"   // dark grey

// ---------------------------------------------------------------------
//  Joint calibration
//    channel      : PCA9685 output (0..15)
//    minPulseUs   : pulse width at servo angle = minAngle
//    maxPulseUs   : pulse width at servo angle = maxAngle
//    minAngle/maxAngle : software travel limits (servo degrees)
//    homeAngle    : rest position, used at start-up
//    kinZero      : servo angle at which the joint is at its kinematic zero
//                   (shoulder/elbow/hand: arm straight up / aligned)
//    kinDir       : +1 or -1, direction of positive kinematic rotation
// ---------------------------------------------------------------------
struct JointCalibration {
  uint8_t  channel;
  uint16_t minPulseUs;
  uint16_t maxPulseUs;
  float    minAngle;
  float    maxAngle;
  float    homeAngle;
  float    kinZero;
  int8_t   kinDir;
};

// Factory defaults (config.cpp)
extern const JointCalibration DEFAULT_JOINT_CALIBRATIONS[NUM_JOINTS];
extern const char* const JOINT_NAMES[NUM_JOINTS];

#endif // CONFIG_H
