# BrazoRobotica – Project Information

## 1. Goals and Objectives
- **Primary Goal**: Control a robotic arm to play chess autonomously.
- **Key Objectives**:
  1. Implement precise movement and control of a 6-DOF robotic arm using standard PWM servos driven by a PCA9685 driver.
  2. Implement an embedded modular firmware architecture on an ESP32-CAM board with clear separation of concerns (motion, networking, vision, chess logic).
  3. Provide an integrated, lightweight web-based UI (HTML/CSS/JavaScript) served directly from the ESP32 for real-time manual joint control and status monitoring.
  4. Ensure reliable, low-latency bidirectional communication between the ESP32 and the Web UI using WebSockets.
  5. Provide automated build and flashing tooling via an `arduino-cli` script for rapid deployment.

---

## 2. Hardware Architecture & Pinout

### 2.1 Hardware Components
- **Microcontroller**: ESP32-CAM with MB programmer board (AI-Thinker module, OV2640 camera sensor).
- **PWM Servo Driver**: PCA9685 16-channel 12-bit I2C PWM driver.
- **Power Supply**: Dedicated 5V/6V (3A–5A) power source for servos and driver board; shared common ground with ESP32.
- **Actuators**: 6 standard hobby servos (PWM controlled at 50 Hz).

### 2.2 Joint & Servo Mapping (PCA9685 Channels 0–5)
Even if the degrees are defined here, these msut be easily configurable in the web ui and must be saved in the non-volatile memory of the ESP32. The configuration allows one to define the range of motion for each joint and the position of each joint when the arm is at rest. The angles are defined in degrees. At startup the servo motor is moved to its home position.
| Joint ID | Name | Rotation Axis | Mechanical Limits / Description | PCA9685 Channel |
| :---: | :--- | :---: | :--- | :---: |
| **0** | Articulation Base | Z-axis | 360° continuous / unrestricted rotation | Channel 0 |
| **1** | Shoulder | X-axis | Limited range (e.g. 0°–180°) | Channel 1 |
| **2** | Elbow | X-axis | Limited range (e.g. 0°–180°) | Channel 2 |
| **3** | Wrist | Y-axis | 360° unrestricted rotation | Channel 3 |
| **4** | Hand | X-axis | Limited range (e.g. 0°–180°) | Channel 4 |
| **5** | Gripper | Z-axis | Clamp open/close angle limits | Channel 5 |

### 2.3 I2C Pin Assignment & Wiring (ESP32-CAM to PCA9685)
> [!IMPORTANT]
> The ESP32-CAM uses most GPIOs for the OV2640 camera and onboard microSD card. 
> The firmware in `config.h` configures the dedicated I2C bus as follows:
> - **SDA**: `GPIO 14`
> - **SCL**: `GPIO 15`
> - **I2C Clock**: Standard `100 kHz` (`100000` Hz) for signal integrity over jumper wires.
> - **PCA9685 Address**: Default `0x40`.
>
> **PCA9685 6-Pin Header Connection Table**:
> | PCA9685 Pin | Pin Order | Connect To | Function |
> | :--- | :---: | :--- | :--- |
> | **GND** | 1 | ESP32-CAM `GND` | Common logic ground |
> | **OE** | 2 | Leave disconnected (or GND) | Output Enable (active LOW; pulled LOW on-board) |
> | **SCL** | 3 | ESP32-CAM `GPIO 15` | I2C Clock line |
> | **SDA** | 4 | ESP32-CAM `GPIO 14` | I2C Data line |
> | **VCC** | 5 | ESP32-CAM `3.3V` | Logic power for PCA9685 IC |
> | **V+** | 6 | External PSU (+) (optional) | Output power rail tap |
>
> **Servo Power Terminal (Screw Block)**:
> - **V+**: External 5V/6V Power Supply (+) (3A–5A).
> - **GND**: External 5V/6V Power Supply (-) (tied to common ground).
>
> *(Note: Do NOT power servos from the ESP32-CAM 3.3V or 5V pins. VCC powers the chip logic; V+ powers the servos).*

---

## 3. Firmware Architecture & Modular Design

The firmware is designed as an Arduino sketch divided into clean C++ header and implementation units. All source files reside in `arduino/BrazoRobotica/` so they can be opened as tabs in the Arduino IDE or compiled via `arduino-cli`.

```
BrazoRobotica/
├─ arduino/
│   └─ BrazoRobotica/
│       ├─ BrazoRobotica.ino      # Main sketch entry point (setup, loop, task orchestration)
│       ├─ config.h               # Pin definitions, Wi-Fi AP parameters, joint calibration structs
│       ├─ pwm_driver.h           # PCA9685 driver interface & joint motion smoothing
│       ├─ pwm_driver.cpp         # PCA9685 initialization, calibration mapping, and servo control
│       ├─ wifi_manager.h         # Wi-Fi SoftAP and web server/WebSocket interface
│       ├─ wifi_manager.cpp       # AP configuration, HTTP route handlers, WebSocket event loop
│       ├─ camera_interface.h     # Camera initialization and streaming interface
│       ├─ camera_interface.cpp   # OV2640 driver configuration, MJPEG stream handler, frame hook
│       ├─ chess_engine.h         # Chess game state and algorithm interface
│       ├─ chess_engine.cpp       # Board representation, placeholder move generator, vision hooks
│       └─ web_ui.h               # HTML/CSS/JS frontend embedded in PROGMEM (raw string literal)
├─ scripts/
│   └─ build_and_flash.ps1        # PowerShell script using arduino-cli to compile and upload
├─ docs/                          # Technical diagrams, hardware pinouts, and references
└─ ProjectInformation.md          # This architecture and design document
```

### 3.1 Module Responsibilities

1. **`BrazoRobotica.ino` (Main Orchestrator)**:
   - Initializes Serial debug logging.
   - Calls subsystem initialization (`config`, `pwm_driver`, `camera_interface`, `wifi_manager`, `chess_engine`).
   - Runs periodic tasks in `loop()` (servicing WebSocket clients, background kinematic/motor smoothing, and safety checks).

2. **`config.h` (Centralized Configuration)**:
   - Stores Wi-Fi AP credentials (SSID: `BrazoRobotica_AP`, Password: `robotarmpass`).
   - Defines I2C pins (`I2C_SDA_PIN`, `I2C_SCL_PIN`) and PCA9685 frequency (50 Hz).
   - Defines `struct JointCalibration`:
     ```cpp
     struct JointCalibration {
       uint8_t channel;
       uint16_t minPulseUs;
       uint16_t maxPulseUs;
       float minAngle;
       float maxAngle;
       float homeAngle;
     };
     ```
   - Defines default calibration array for all 6 joints.

3. **`pwm_driver` (`pwm_driver.h` / `pwm_driver.cpp`)**:
   - Manages communication with the PCA9685 driver over I2C (using `Wire.h`).
   - Converts desired angle (degrees) to pulse tick length (0–4095) using joint-specific calibration.
   - Enforces software mechanical limits to protect physical joints.
   - Provides methods:
     - `bool initPWM()`
     - `void setJointAngle(uint8_t jointId, float angle)`
     - `void setAllJointAngles(const float angles[6])`
     - `float getJointAngle(uint8_t jointId)`
     - `void moveHome()`

4. **`wifi_manager` (`wifi_manager.h` / `wifi_manager.cpp`)**:
   - Configures the ESP32 in Access Point (SoftAP) mode (`192.168.4.1`).
   - Serves the embedded Web UI at HTTP `GET /`.
   - Hosts the WebSocket server at `/ws` for real-time bidirectional communication.
   - Parses incoming JSON command payloads using `ArduinoJson` and triggers motion methods in `pwm_driver`.
   - Broadcasts joint telemetry updates to connected clients.

5. **`camera_interface` (`camera_interface.h` / `camera_interface.cpp`)**:
   - Configures and initializes the OV2640 camera with AI-Thinker pin configurations.
   - Provides an HTTP MJPEG stream handler at `GET /stream`.
   - Exposes placeholder hook `camera_fb_t* captureFrame()` for future computer-vision piece detection.

6. **`chess_engine` (`chess_engine.h` / `chess_engine.cpp`)**:
   - Maintains virtual 8x8 chessboard representation (FEN or 64-square array).
   - Contains placeholder move generation algorithms (`generateBestMove()`, `validateMove()`).
   - Contains placeholder hooks for vision-based piece position updates.

7. **`web_ui.h` (Embedded Single-Page Application)**:
   - Contains minified, standalone HTML5, CSS3, and JavaScript stored as a `const char WEB_UI_HTML[] PROGMEM`.
   - No external CDN or internet connection required; completely offline.

---

## 4. Communication Protocol & WebSocket Payloads

All real-time control commands between the browser UI and the ESP32 run over WebSockets (`ws://192.168.4.1/ws`). Payloads use structured JSON.

### 4.1 Client-to-ESP32 Messages (Control Commands)

- **Set Single Joint Angle**:
  ```json
  {
    "type": "set_joint",
    "id": 1,
    "angle": 45.0
  }
  ```
- **Set All Joint Angles Simultaneously**:
  ```json
  {
    "type": "set_all",
    "angles": [90.0, 45.0, 120.0, 180.0, 60.0, 30.0]
  }
  ```
- **Command Arm to Home Position**:
  ```json
  {
    "type": "home"
  }
  ```

### 4.2 ESP32-to-Client Messages (Telemetry & Status Feedback)

- **Joint State Broadcast**:
  ```json
  {
    "type": "status",
    "angles": [90.0, 45.0, 120.0, 180.0, 60.0, 30.0],
    "connected": true,
    "uptime_ms": 34500
  }
  ```

---

## 5. Web UI Specifications

The Web UI is delivered as a sleek, modern, mobile-friendly control dashboard:
- **Joint Sliders**: 6 responsive range sliders (one per joint) with real-time numeric angle indicators (`0°` to `180°` or `360°` depending on joint limits).
- **Direct Numeric Input**: Number fields next to sliders allowing precise manual entry.
- **Connection Indicator**: Visual badge showing WebSocket connection status (Connected / Reconnecting).
- **Auto-Reconnect**: Robust JavaScript WebSocket client that automatically reconnects if Wi-Fi drops.

---

## 6. Build, Deployment, and Automation

### 6.1 Prerequisites
- **Arduino CLI** installed and available in system PATH.
- **ESP32 Arduino Core** installed (`esp32:esp32`).
- **Required Arduino Libraries**:
  - `Adafruit PWM Servo Driver Library`
  - `ArduinoJson` (v6 or v7)
  - `ESPAsyncWebServer` & `AsyncTCP` (or standard `WebServer` + `WebSocketsServer`)

### 6.2 Automation Script (`scripts/build_and_flash.ps1`)
The repository includes a PowerShell build script to streamline compilation and flashing to the ESP32-CAM MB:
- Automatically detects connected serial COM ports (e.g. CH340 / CP2102).
- Compiles the sketch targeting FQBN `esp32:esp32:esp32cam`.
- Uploads the compiled binary directly over the selected COM port.
- Optional flag to open the serial monitor after flashing.

Usage example:
```powershell
.\scripts\build_and_flash.ps1 -Port COM3 -CompileAndUpload
```

---

## 7. Running the System

1. **Power**: Connect the 5V power supply to the PCA9685 servo driver and ESP32.
2. **Flash**: Upload the firmware to the ESP32-CAM using `scripts/build_and_flash.ps1` or the Arduino IDE.
3. **Connect Wi-Fi**: On your computer, tablet, or smartphone, connect to the Wi-Fi network:
   - **SSID**: `BrazoRobotica_AP`
   - **Password**: `robotarmpass`
4. **Open UI**: Open any modern browser and navigate to `http://192.168.4.1`.
5. **Control**: Move sliders to control joints in real-time.

---

## 8. Future Work & Implementation Guidelines

> [!CAUTION]
> Before implementing features in this section, ask the user for permission and create a detailed step-by-step implementation plan.

1. **Real-time Computer Vision & Movement Engine**:
   - Utilize the OV2640 camera feed to detect chessboard corners and square coordinates using perspective transformation.
   - Implement piece recognition (identifying square occupancy and piece colors).
   - Implement inverse kinematics (IK) in the movement engine to translate 3D Cartesian coordinates `(X, Y, Z)` to joint servo angles.
2. **Autonomous Chess Engine Integration**:
   - Integrate an embedded micro chess engine (such as Micro-Max or a lightweight chess move validator).
   - Alternatively, support Web-assisted calculation where the Web UI queries an external chess API / Stockfish worker and dispatches computed coordinates back to the arm.
3. **Hardware Assembly & Calibration Guide**:
   - Provide visual schematics and step-by-step assembly instructions in `hardware/README.md`.

