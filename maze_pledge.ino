/*
 * Line MAZE SOLVER with Pledge Algorithm + MPU6050
 * Using Maker Line (5 sensors + D6 center) & Maker Robo ESP32
 * Tuned for N20 Motors (600RPM, 30:1 gear ratio, 6V)
 * MPU6050 Gyroscope-Based Closed-Loop Turns
 *
 * N20 Motor Specs:
 *   Rated voltage: 7VDC (3V-9V range)
 *   Free run speed @ 6V: 600 RPM
 *   Gear ratio: 30:1
 *
 * Algorithm: Pledge Algorithm with Angle Turn Counter
 *   - Maintains angleCounter (cumulative sum of all turns in degrees)
 *   - When angleCounter == 0: go straight (preferred heading)
 *   - When angleCounter != 0: follow left-wall rule (L > S > R > B)
 *   - angleCounter returning to 0 marks end of dead branch → eliminate it
 *   - Run 1: Explores maze using Pledge, records & optimizes path
 *   - Run 2: Follows optimized shortest path
 *
 * D6 (Bottom-Middle Extra Center Sensor):
 *   - Mounted at the very center bottom of the robot
 *   - Weight = 0 (no steering bias, stabilizes error denominator)
 *   - Confirms line under center at intersections (filters false triggers)
 *
 * Intersection Detection (5 digital sensors + D6):
 *   All on (11111)       = Cross / T-intersection
 *   Left+Center (111xx)  = Left turn available
 *   Right+Center (xx111) = Right turn available
 *   All off (00000)      = Dead end → U-turn
 *
 * Path Optimization:
 *   Angle-based dead branch elimination (Pledge angle counter)
 *   Traditional x-B-y rules:
 *   LBR → B    LBS → R    LBL → S
 *   SBL → R    SBS → B    RBL → B
 *
 * Maker Line Sensor Pins:
 *   D1 (Leftmost):  32
 *   D2 (Left):      39
 *   D3 (Center):    21
 *   D4 (Right):     22
 *   D5 (Rightmost): 25
 *   D6 (Bottom-Middle): 26  ← extra center sensor for stability
 *
 * MPU6050 Wiring:
 *   SDA → GPIO 17
 *   SCL → GPIO 16
 *   VCC → 3.3V
 *   GND → GND
 *   AD0 → GND (address 0x68)
 */

#include "BluetoothSerial.h"
#include <Wire.h>

BluetoothSerial SerialBT;

// ==================== PIN DEFINITIONS ====================
#define SENSOR_D1 32 // Leftmost
#define SENSOR_D2 39 // Left
#define SENSOR_D3 21 // Center (Maker Line D3)
#define SENSOR_D4 22 // Right
#define SENSOR_D5 25 // Rightmost
#define SENSOR_D6 26 // Bottom-middle (extra center sensor for stability)
#define D6_ENABLED true
#define BT_DEVICE_NAME "MazeBot_N20_Debug"
#define DEBUG_STREAM_INTERVAL_MS 200

// Motor Driver Pins (Maker Robo ESP32)
#define MOTOR_LEFT_PWM 27
#define MOTOR_LEFT_DIR 14
#define MOTOR_RIGHT_PWM 13
#define MOTOR_RIGHT_DIR 12

// MPU6050 I2C Pins
#define MPU_SDA 17
#define MPU_SCL 16

// LEDC PWM Configuration
// NOTE: 20kHz is too high for N20 motors + MX1508 H-bridge — motor inductance
// prevents current buildup per cycle, drastically reducing effective torque.
// 1kHz delivers full power (slight audible whine). Use 5000 if noise bothers
// you.
#define PWM_FREQ 1000
#define PWM_RESOLUTION 10
#define PWM_MAX 1023

// ==================== MPU6050 REGISTER DEFINITIONS ====================
#define MPU_ADDR 0x68
#define MPU_REG_SMPLRT_DIV 0x19
#define MPU_REG_CONFIG 0x1A
#define MPU_REG_GYRO_CONFIG 0x1B
#define MPU_REG_GYRO_ZOUT_H 0x47
#define MPU_REG_GYRO_ZOUT_L 0x48
#define MPU_REG_SIGNAL_PATH_RESET 0x68
#define MPU_REG_USER_CTRL 0x6A
#define MPU_REG_PWR_MGMT_1 0x6B
#define MPU_REG_WHO_AM_I 0x75

// ==================== MPU6050 CONFIGURATION ====================
#define GYRO_FS_SEL 1
#define GYRO_FS_SEL_REG_VAL 0x08
#define GYRO_SENSITIVITY 65.5
#define GYRO_DLPF_CFG 3
#define GYRO_SAMPLE_RATE_DIV 1

// ==================== TURN ANGLE CONTROL ====================
#define TURN_ANGLE_90 90.0
#define TURN_ANGLE_180 180.0
#define ANGLE_TOLERANCE 2.0
#define ANGLE_SLOW_ZONE 5.0
#define TURN_SLOW_FACTOR 0.5
#define GYRO_TURN_SIGN -1

// ==================== CALIBRATION ====================
#define GYRO_CALIBRATION_SAMPLES 500

// ==================== PID PARAMETERS ====================
// Gains scaled for BASE_SPEED=600. Error range is [-4,+4], so:
//   KP=50 → max P-correction = ±200 (33% of 600) — strong enough to steer
//   KD=55 → damps oscillation from the higher KP
// Original KP=10 only gave ±40 correction (6.7% of 600) — far too weak.
#define KP 50.0
#define KI 0.0
#define KD 55.0
#define INTEGRAL_MAX 100

// ==================== SPEED SETTINGS ====================
#define BASE_SPEED 600 // Line following speed
#define TURN_SPEED 160 // Speed during turns
#define MAX_SPEED 1023
#define MIN_SPEED 80
#define FORWARD_NUDGE 120  // Slow speed for nudging past intersections
#define RECOVERY_SPEED 140 // Speed for D6 line-loss recovery

// ==================== TIMING ====================
#define INTERSECTION_FORWARD_MS 120 // Drive past intersection before checking
#define DEAD_END_BRAKE_MS 50        // Brake before U-turn
#define INTERSECTION_DEBOUNCE                                                  \
  3 // Consecutive reads needed to confirm intersection
#define MIN_INTERSECTION_GAP_MS                                                \
  400                              // Minimum ms between two intersection events
#define GYRO_TURN_TIMEOUT_MS 2000  // Safety timeout for gyro-based turns
#define GYRO_DT_MAX_US 10000       // Max dt guard for micros() wrap
#define D6_RECOVERY_TIMEOUT_MS 500 // Timeout for D6 line-loss recovery
#define EXIT_CONFIRM_MS 800        // All-black hold time to confirm exit
#define EXIT_GRACE_MS 150          // Brief sensor-noise gap allowed during exit confirm
#define PID_INTERVAL_US 2000       // Fixed PID rate: 2ms = 500Hz

// ==================== SENSOR WEIGHTS ====================
const int WEIGHT_D1 = -4;
const int WEIGHT_D2 = -2;
const int WEIGHT_D3 = 0;
const int WEIGHT_D4 = 2;
const int WEIGHT_D5 = 4;
const int WEIGHT_D6 = 0; // Extra center sensor: no steering bias

// ==================== PATH STORAGE ====================
#define MAX_PATH 200 // Maximum number of turns to store
char path[MAX_PATH]; // Stores: 'L', 'R', 'S', 'B' (back/u-turn)
int pathIndex = 0;

// ==================== PLEDGE ANGLE COUNTER ====================
int pledgeAngleCounter =
    0; // Cumulative turn angle (degrees): L=-90, R=+90, B=+180
bool pledgeWallFollowing = false; // true when in wall-following mode

// Angle counter history for dead branch detection
int angleAtTurn[MAX_PATH];    // angleCounter value BEFORE each recorded turn
int angleAfterTurn[MAX_PATH]; // angleCounter value AFTER each recorded turn

// ==================== STATE ====================
bool mazeFinished = false;
bool robotRunning = false;
bool debugStreamEnabled = true;
unsigned long lastDebugStreamMs = 0;
unsigned long allBlackStartMs = 0;
unsigned long lastAllBlackMs = 0;  // Last time all 5 sensors were black
unsigned long lastIntersectionMs =
    0;                            // Timestamp of last confirmed intersection
int intersectionConfirmCount = 0; // Consecutive reads with s1/s5 active
bool intersectionArmed = true;    // Must re-center on line before next intersection
float headingAtStart = 0.0;       // Yaw when run started
unsigned long lastLineSeenMs = 0; // Last time a valid line pattern was seen
unsigned long lastPidMicros = 0;  // Last PID execution timestamp (fixed-rate)
#define LINE_LOST_TIMEOUT_MS                                                   \
  500 // If no valid line for this long, enter recovery
#define HEADING_DRIFT_LIMIT                                                    \
  45.0 // Max yaw drift from start heading during straight following
String btInputBuffer = "";

// Run mode
enum RunMode {
  RUN_EXPLORE, // First run - Pledge algorithm + record
  RUN_SOLVED   // Second run - follow optimized path
};

RunMode runMode = RUN_EXPLORE;
int solvedPathIndex = 0; // Index into path[] during solved run

// Sensor readings
int s1, s2, s3, s4, s5, s6;
float lastError = 0;
float integral = 0;

// ==================== MPU6050 GLOBALS ====================
int16_t gyroZOffset = 0;          // Calibrated zero-rate bias (raw LSB)
float yawAngle = 0.0;             // Integrated yaw angle in degrees
unsigned long lastGyroMicros = 0; // Timestamp of last gyro integration

// ==================== MPU6050 I2C HELPERS ====================
void writeMPU6050Register(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

uint8_t readMPU6050Register(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
  return Wire.read();
}

// ==================== MPU6050 SETUP & CALIBRATION ====================
void setupMPU6050() {
  uint8_t whoAmI = readMPU6050Register(MPU_REG_WHO_AM_I);
  Serial.print("MPU6050 WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);
  if (whoAmI != 0x68) {
    Serial.println("ERROR: MPU6050 not found! Check wiring (SDA=17, SCL=16).");
    Serial.println("HALTING.");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("MPU6050 detected OK.");

  writeMPU6050Register(MPU_REG_PWR_MGMT_1, 0x00);
  delay(100);
  writeMPU6050Register(MPU_REG_PWR_MGMT_1, 0x01);
  writeMPU6050Register(MPU_REG_SIGNAL_PATH_RESET, 0x07);
  delay(100);
  writeMPU6050Register(MPU_REG_USER_CTRL, 0x00);
  writeMPU6050Register(MPU_REG_CONFIG, GYRO_DLPF_CFG);
  writeMPU6050Register(MPU_REG_GYRO_CONFIG, GYRO_FS_SEL_REG_VAL);
  writeMPU6050Register(MPU_REG_SMPLRT_DIV, GYRO_SAMPLE_RATE_DIV);

  Serial.println("MPU6050 configured: FS=+-500dps, DLPF=42Hz, SR=500Hz");

  Serial.print("Calibrating gyro (");
  Serial.print(GYRO_CALIBRATION_SAMPLES);
  Serial.print(" samples)... ");

  long sum = 0;
  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_REG_GYRO_ZOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, (uint8_t)true);
    int16_t raw = ((int16_t)Wire.read() << 8) | Wire.read();
    sum += raw;
    delay(2);
  }

  gyroZOffset = (int16_t)(sum / GYRO_CALIBRATION_SAMPLES);
  Serial.print("Done. Z-offset = ");
  Serial.println(gyroZOffset);

  yawAngle = 0.0;
  lastGyroMicros = micros();
}

// ==================== GYRO READING ====================
int16_t readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, (uint8_t)true);
  int16_t raw = ((int16_t)Wire.read() << 8) | Wire.read();
  return raw - gyroZOffset;
}

// ==================== YAW INTEGRATION ====================
void updateYaw() {
  unsigned long now = micros();
  unsigned long elapsed = now - lastGyroMicros;
  if (elapsed > GYRO_DT_MAX_US) {
    elapsed = GYRO_DT_MAX_US;
  }
  float dt = (float)elapsed / 1000000.0;
  lastGyroMicros = now;

  int16_t gz = readGyroZ();
  float degreesPerSec = (float)gz / GYRO_SENSITIVITY;
  yawAngle += degreesPerSec * dt;
}

// ==================== GYRO-BASED TURN FUNCTIONS ====================
void gyroTurn(float targetDeg, int motorLeft, int motorRight) {
  float startAngle = yawAngle;
  float absTarg = fabs(targetDeg);
  int slowSpeed = (int)(TURN_SPEED * TURN_SLOW_FACTOR);
  unsigned long startMs = millis();

  setMotorSpeeds(motorLeft, motorRight);

  while (true) {
    updateYaw();
    float angleTurned = fabs(yawAngle - startAngle);

    if ((millis() - startMs) > GYRO_TURN_TIMEOUT_MS) {
      Serial.print("WARN: Gyro turn timeout! Achieved: ");
      Serial.print(angleTurned, 1);
      Serial.print("deg / ");
      Serial.print(absTarg, 1);
      Serial.println("deg");
      break;
    }

    if (angleTurned >= (absTarg - ANGLE_SLOW_ZONE)) {
      setMotorSpeeds(
          (motorLeft > 0) ? slowSpeed : ((motorLeft < 0) ? -slowSpeed : 0),
          (motorRight > 0) ? slowSpeed : ((motorRight < 0) ? -slowSpeed : 0));
    }

    if (angleTurned >= (absTarg - ANGLE_TOLERANCE)) {
      break;
    }
  }

  stopMotors();
  delay(30);

  float achieved = fabs(yawAngle - startAngle);
  Serial.print("Turn complete: ");
  Serial.print(achieved, 1);
  Serial.print("deg (target ");
  Serial.print(absTarg, 1);
  Serial.println("deg)");

  // Post-turn: fine-tune onto the line using sensors
  // Gyro gets us close, sensors lock us onto the line
  readSensors();
  if (s2 == 0 && s3 == 0 && s4 == 0) {
    // Sensors don't see the line yet - keep rotating slowly in same direction
    int sweepLeft =
        (motorLeft > 0) ? slowSpeed : ((motorLeft < 0) ? -slowSpeed : 0);
    int sweepRight =
        (motorRight > 0) ? slowSpeed : ((motorRight < 0) ? -slowSpeed : 0);
    setMotorSpeeds(sweepLeft, sweepRight);

    unsigned long sweepStart = millis();
    while ((millis() - sweepStart) < 300) { // Max 300ms fine-tune
      readSensors();
      if (s2 == 1 || s3 == 1 || s4 == 1) {
        break; // Found the line
      }
      delay(2);
    }
    stopMotors();
    Serial.println("Post-turn sensor fine-tune applied");
  }
}

void turnLeft90() {
  float targetDeg = GYRO_TURN_SIGN * TURN_ANGLE_90;
  gyroTurn(targetDeg, -TURN_SPEED, TURN_SPEED);
}

void turnRight90() {
  float targetDeg = -GYRO_TURN_SIGN * TURN_ANGLE_90;
  gyroTurn(targetDeg, TURN_SPEED, -TURN_SPEED);
}

void uTurn() {
  float targetDeg = GYRO_TURN_SIGN * TURN_ANGLE_180;
  gyroTurn(targetDeg, -TURN_SPEED, TURN_SPEED);
}

// ==================== BLUETOOTH HELPERS ====================
void sendBoth(const String &msg) {
  Serial.println(msg);
  if (SerialBT.hasClient()) {
    SerialBT.println(msg);
  }
}

void sendStatus() {
  String status = "STATUS:";
  status += "RUN=" + String(robotRunning ? 1 : 0);
  status += ",MODE=" + String(runMode == RUN_EXPLORE ? "EXPLORE" : "SOLVED");
  status += ",FINISHED=" + String(mazeFinished ? 1 : 0);
  status += ",PATH_LEN=" + String(pathIndex);
  status += ",SOLVED_IDX=" + String(solvedPathIndex);
  status += ",ANGLE=" + String(pledgeAngleCounter);
  status += ",WALL=" + String(pledgeWallFollowing ? 1 : 0);
  sendBoth(status);
}

void sendDebugSnapshot(const String &tag) {
  String debug = "DEBUG:" + tag;
  debug +=
      ",S=" + String(s1) + String(s2) + String(s3) + String(s4) + String(s5);
  debug += ",D6=" + String(s6);
  debug += ",E=" + String(lastError, 2);
  debug += ",YAW=" + String(yawAngle, 1);
  debug += ",PLEDGE=" + String(pledgeAngleCounter);
  debug += ",PI=" + String(pathIndex);
  debug += ",RI=" + String(solvedPathIndex);
  sendBoth(debug);
}

void resetForNewExplore() {
  runMode = RUN_EXPLORE;
  solvedPathIndex = 0;
  pathIndex = 0;
  memset(path, 0, MAX_PATH);
  memset(angleAtTurn, 0, sizeof(angleAtTurn));
  memset(angleAfterTurn, 0, sizeof(angleAfterTurn));
  pledgeAngleCounter = 0;
  pledgeWallFollowing = false;
  mazeFinished = false;
  intersectionConfirmCount = 0;
  intersectionArmed = true;
  lastIntersectionMs = 0;
  lastError = 0;
  integral = 0;
  yawAngle = 0;
  lastGyroMicros = micros();
  lastPidMicros = micros();
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) {
    return;
  }

  if (cmd == "S" || cmd == "START") {
    if (mazeFinished && runMode == RUN_SOLVED) {
      sendBoth("Restarting with a fresh EXPLORE run.");
      resetForNewExplore();
    }
    if (runMode == RUN_SOLVED) {
      solvedPathIndex = 0;
      mazeFinished = false;
    }
    robotRunning = true;
    allBlackStartMs = 0;
    lastAllBlackMs = 0;
    intersectionConfirmCount = 0;
    intersectionArmed = true;
    lastIntersectionMs = 0;
    lastError = 0;
    integral = 0;
    yawAngle = 0.0;
    headingAtStart = 0.0;
    lastLineSeenMs = millis();
    lastGyroMicros = micros();
    lastPidMicros = micros();
    sendBoth(">>> STARTED <<<");
    sendStatus();
  } else if (cmd == "X" || cmd == "STOP") {
    robotRunning = false;
    allBlackStartMs = 0;
    lastAllBlackMs = 0;
    intersectionConfirmCount = 0;
    stopMotors();
    sendBoth(">>> STOPPED <<<");
    sendStatus();
  } else if (cmd == "R" || cmd == "RESET") {
    robotRunning = false;
    allBlackStartMs = 0;
    intersectionConfirmCount = 0;
    lastIntersectionMs = 0;
    stopMotors();
    resetForNewExplore();
    sendBoth(">>> RESET DONE (EXPLORE MODE) <<<");
    sendStatus();
  } else if (cmd == "D") {
    debugStreamEnabled = !debugStreamEnabled;
    sendBoth(String("Debug stream: ") + (debugStreamEnabled ? "ON" : "OFF"));
  } else if (cmd == "P") {
    String pathMsg = "PATH:";
    for (int i = 0; i < pathIndex; i++) {
      pathMsg += path[i];
      if (i < pathIndex - 1)
        pathMsg += "-";
    }
    sendBoth(pathMsg);
  } else if (cmd == "?" || cmd == "STATUS") {
    sendStatus();
  } else if (cmd == "H" || cmd == "HELP") {
    sendBoth("=== COMMANDS ===");
    sendBoth("START/S  : Start run (Run 2 if solved path exists)");
    sendBoth("STOP/X   : Stop motors");
    sendBoth("RESET/R  : Reset to fresh explore run");
    sendBoth("STATUS/? : Show status");
    sendBoth("D        : Toggle periodic debug stream");
    sendBoth("P        : Print current path");
    sendBoth("HELP/H   : Show this help");
    sendBoth("===============");
  } else {
    sendBoth("Unknown command: " + cmd);
  }
}

void handleBluetoothInput() {
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (btInputBuffer.length() > 0) {
        processCommand(btInputBuffer);
        btInputBuffer = "";
      }
    } else {
      btInputBuffer += c;
    }
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  SerialBT.begin(BT_DEVICE_NAME);
  Serial.println("=== MAZE SOLVER - Pledge Algorithm ===");
  Serial.println("N20 Motors (600RPM 30:1) + Maker Robo ESP32");
  Serial.println("MPU6050 Gyro Turns + D6 Center Safety Sensor");
  Serial.print("Bluetooth Name: ");
  Serial.println(BT_DEVICE_NAME);

  // Initialize I2C for MPU6050
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000);

  // Initialize and calibrate MPU6050
  setupMPU6050();

  // Initialize sensor pins
  pinMode(SENSOR_D1, INPUT);
  pinMode(SENSOR_D2, INPUT);
  pinMode(SENSOR_D3, INPUT);
  pinMode(SENSOR_D4, INPUT);
  pinMode(SENSOR_D5, INPUT);
  pinMode(SENSOR_D6, INPUT);

  // Initialize motor direction pins
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);

  // Setup LEDC for PWM motor control
  ledcAttach(MOTOR_LEFT_PWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_RIGHT_PWM, PWM_FREQ, PWM_RESOLUTION);

  stopMotors();

  pathIndex = 0;
  memset(path, 0, MAX_PATH);
  pledgeAngleCounter = 0;
  pledgeWallFollowing = false;
  robotRunning = false;
  mazeFinished = false;
  debugStreamEnabled = true;
  lastDebugStreamMs = millis();

  sendBoth("Run mode: EXPLORE (Pledge algorithm)");
  // ---- Sensor diagnostic: verify polarity (expect 1=on-line, 0=off-line) ----
  readSensors();
  Serial.print("Sensor check (place robot ON line): D1=");
  Serial.print(s1);
  Serial.print(" D2=");
  Serial.print(s2);
  Serial.print(" D3=");
  Serial.print(s3);
  Serial.print(" D4=");
  Serial.print(s4);
  Serial.print(" D5=");
  Serial.print(s5);
  Serial.print(" D6=");
  Serial.println(s6);
  Serial.println("Expected: center sensor(s)=1, outer=0. If inverted, flip "
                 "Maker Line jumper.");

  lastGyroMicros = micros();
  lastPidMicros = micros();
  yawAngle = 0.0;
  sendBoth("Ready. Send START (or S) over Bluetooth to begin.");
}

// ==================== MAIN LOOP ====================
void loop() {
  // --- Always run (no gating) ---
  handleBluetoothInput();
  readSensors();
  updateYaw(); // Keep yaw integrated as fast as possible

  if (!robotRunning) {
    stopMotors();
    delay(1); // Minimal idle wait to avoid CPU spin
    return;
  }

  // ── Exit detection ──────────────────────────────────────────────
  // All 5 sensors black = potential finish zone.
  // Strategy: start timer only when NOT near a recent intersection
  // (avoids false trigger at cross-junctions). Once timer starts,
  // slow to a crawl and SKIP followLineAndDetect so intersection
  // handling can't reset the timer. Allow brief sensor noise gaps.
  bool allBlack = (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1);
  if (allBlack) lastAllBlackMs = millis();

  // Start timer: require allBlack AND >300ms since last intersection
  bool nearIntersection = (millis() - lastIntersectionMs) < 300;
  if (allBlack && !nearIntersection && allBlackStartMs == 0) {
    allBlackStartMs = millis();
    sendBoth("Exit candidate: all sensors black, confirming...");
  }

  // Timer active — manage confirmation
  if (allBlackStartMs > 0) {
    // Confirmed: sustained allBlack for EXIT_CONFIRM_MS
    if ((millis() - allBlackStartMs) >= EXIT_CONFIRM_MS) {
      stopMotors();
      sendBoth("Exit confirmed.");
      handleFinish();
      return;
    }
    // Sensors left allBlack — allow brief gap (noise on exit-zone edge)
    if (!allBlack && (millis() - lastAllBlackMs) > EXIT_GRACE_MS) {
      allBlackStartMs = 0; // Gap too long, reset — resume normal control
    } else {
      // Still confirming: creep forward, skip control loop entirely
      // (prevents intersection handling from disrupting timer)
      setMotorSpeeds(FORWARD_NUDGE, FORWARD_NUDGE);
      return;
    }
  }
  // ── End exit detection ────────────────────────────────────────

  // --- Fixed-rate control loop (every PID_INTERVAL_US) ---
  // Sensors & gyro update every loop (~10kHz+), but control fires at
  // a rock-steady 500Hz so PID gains behave identically every cycle.
  unsigned long nowUs = micros();
  unsigned long elapsed = nowUs - lastPidMicros;
  if (elapsed >= PID_INTERVAL_US) {
    // Compute dt-ratio ONCE here — valid even if followLineAndDetect
    // takes an early return (search/intersection/dead-end).
    float dtUs = (float)elapsed;
    if (dtUs <= 0 || dtUs > 50000) dtUs = PID_INTERVAL_US; // guard wrap
    float dtRatio = dtUs / (float)PID_INTERVAL_US;          // 1.0 at 2ms
    lastPidMicros = nowUs; // Always update — prevents stale timestamps
    followLineAndDetect(dtRatio);
  }

  // Debug output (non-blocking, own timer)
  if (debugStreamEnabled &&
      (millis() - lastDebugStreamMs) >= DEBUG_STREAM_INTERVAL_MS) {
    sendDebugSnapshot("RUN");
    lastDebugStreamMs = millis();
  }
  // No delay() — loop free-runs for maximum sensor & gyro update rate
}

// ==================== SENSOR FUNCTIONS ====================
void readSensors() {
  s1 = digitalRead(SENSOR_D1);                  // Leftmost
  s2 = digitalRead(SENSOR_D2);                  // Left
  s3 = digitalRead(SENSOR_D3);                  // Center
  s4 = digitalRead(SENSOR_D4);                  // Right
  s5 = digitalRead(SENSOR_D5);                  // Rightmost
  s6 = digitalRead(SENSOR_D6); // Bottom-middle center sensor
}

// ==================== D6 LINE LOSS RECOVERY ====================
void recoverFromLineLoss() {
  Serial.println("WARN: D6 line lost! Recovering...");
  unsigned long startMs = millis();
  int searchDir = (lastError >= 0) ? 1 : -1;

  // Search in last known direction
  while ((millis() - startMs) < D6_RECOVERY_TIMEOUT_MS) {
    readSensors();
    if (s6 == 1 || s2 == 1 || s3 == 1 || s4 == 1) {
      Serial.println("Line recovered.");
      stopMotors();
      return;
    }
    if (searchDir > 0) {
      setMotorSpeeds(RECOVERY_SPEED, -RECOVERY_SPEED);
    } else {
      setMotorSpeeds(-RECOVERY_SPEED, RECOVERY_SPEED);
    }
    delay(2);
  }

  // Try opposite direction
  searchDir = -searchDir;
  startMs = millis();
  while ((millis() - startMs) < D6_RECOVERY_TIMEOUT_MS) {
    readSensors();
    if (s6 == 1 || s2 == 1 || s3 == 1 || s4 == 1) {
      Serial.println("Line recovered (reverse search).");
      stopMotors();
      return;
    }
    if (searchDir > 0) {
      setMotorSpeeds(RECOVERY_SPEED, -RECOVERY_SPEED);
    } else {
      setMotorSpeeds(-RECOVERY_SPEED, RECOVERY_SPEED);
    }
    delay(2);
  }

  stopMotors();
  Serial.println("WARN: D6 recovery failed!");
}

// ==================== VALID LINE CHECK ====================
// A valid line reading has contiguous active sensors (no gaps).
// Patterns like 10101, 10001 are noise/off-track.
bool isValidLinePattern() {
  int count = s1 + s2 + s3 + s4 + s5;
  if (count == 0)
    return (s6 == 1); // D6 alone = line under center
  if (count == 5)
    return true; // all on (junction/exit)
  // Check for gaps: if outer sensor is on, its inner neighbour must also be on
  if (s1 == 1 && s2 == 0)
    return false; // gap between D1 and rest
  if (s5 == 1 && s4 == 0)
    return false; // gap between D5 and rest
  // Check interior gap: s1 or s2 on, then gap, then s4 or s5 on
  if ((s1 == 1 || s2 == 1) && s3 == 0 && (s4 == 1 || s5 == 1))
    return false;
  return true;
}

// ==================== LINE LOSS SEARCH RECOVERY ====================
void searchForLine() {
  sendBoth("WARN: Line lost! Searching...");
  stopMotors();
  delay(20);

  int searchDir = (lastError >= 0) ? 1 : -1;

  // Search in last-error direction first
  unsigned long startMs = millis();
  while ((millis() - startMs) < D6_RECOVERY_TIMEOUT_MS) {
    readSensors();
    if (isValidLinePattern() && (s2 == 1 || s3 == 1 || s4 == 1)) {
      sendBoth("Line recovered (search).");
      stopMotors();
      lastLineSeenMs = millis();
      return;
    }
    if (searchDir > 0)
      setMotorSpeeds(RECOVERY_SPEED, -RECOVERY_SPEED);
    else
      setMotorSpeeds(-RECOVERY_SPEED, RECOVERY_SPEED);
    updateYaw();
    delay(2);
  }

  // Try opposite direction (sweep twice as far)
  searchDir = -searchDir;
  startMs = millis();
  while ((millis() - startMs) < (D6_RECOVERY_TIMEOUT_MS * 2)) {
    readSensors();
    if (isValidLinePattern() && (s2 == 1 || s3 == 1 || s4 == 1)) {
      sendBoth("Line recovered (reverse search).");
      stopMotors();
      lastLineSeenMs = millis();
      return;
    }
    if (searchDir > 0)
      setMotorSpeeds(RECOVERY_SPEED, -RECOVERY_SPEED);
    else
      setMotorSpeeds(-RECOVERY_SPEED, RECOVERY_SPEED);
    updateYaw();
    delay(2);
  }

  // Could not find line — treat as dead end
  stopMotors();
  sendBoth("WARN: Search failed, treating as dead end.");
  handleDeadEnd();
}

// ==================== LINE FOLLOWING WITH INTERSECTION DETECTION ==========
void followLineAndDetect(float dtRatio) {
  // Check if current sensor pattern is valid (contiguous)
  bool validPattern = isValidLinePattern();
  int sensorCount = s1 + s2 + s3 + s4 + s5;

  // Track last time we saw a real line (D6 alone counts as line present)
  if ((validPattern && sensorCount > 0 && sensorCount < 5) || s6 == 1) {
    lastLineSeenMs = millis();
  }

  // Dead end: all 5 maze sensors off
  if (sensorCount == 0) {
    // D6=1 keeps lastLineSeenMs fresh (updated above), so the robot
    // will search rather than dead-end while D6 still sees the line.
    if ((millis() - lastLineSeenMs) < LINE_LOST_TIMEOUT_MS) {
      searchForLine();
    } else {
      handleDeadEnd();
    }
    return;
  }

  // Non-contiguous pattern (e.g. 10101) = off-track noise
  if (!validPattern) {
    // If persists beyond timeout, search for line
    if ((millis() - lastLineSeenMs) >= LINE_LOST_TIMEOUT_MS) {
      searchForLine();
      return;
    }
    // Otherwise keep following with last known error (brief noise)
    setMotorSpeeds(BASE_SPEED, BASE_SPEED);
    return;
  }

  // Re-arm intersection detection once robot is centered on the line
  // (center sensor on, both outermost sensors off, <=3 sensors active)
  if ((s3 == 1 || s6 == 1) && s1 == 0 && s5 == 0 && sensorCount <= 3) {
    intersectionArmed = true;
  }

  // Intersection: require outer+adjacent sensor pair + must be armed
  bool leftDetect = (s1 == 1 && s2 == 1);
  bool rightDetect = (s5 == 1 && s4 == 1);

  // D6 confirms line is under robot center — filters drift from real intersections
  bool centerConfirmed = !D6_ENABLED || (s6 == 1);
  if ((leftDetect || rightDetect) && intersectionArmed && centerConfirmed) {
    intersectionConfirmCount++;
    if (intersectionConfirmCount >= INTERSECTION_DEBOUNCE &&
        (millis() - lastIntersectionMs) >= MIN_INTERSECTION_GAP_MS) {
      intersectionConfirmCount = 0;
      lastIntersectionMs = millis();
      intersectionArmed = false; // Disarm until robot re-centers on line
      headingAtStart =
          yawAngle; // Reset heading reference after each intersection
      handleIntersection(leftDetect, rightDetect);
      return;
    }
  } else {
    intersectionConfirmCount = 0;
  }

  // Normal line following using PID (dtRatio passed from fixed-rate gate)
  int error = calculateError();
  int correction = pidControl(error, dtRatio);

  int leftSpeed = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  applyMinSpeed(leftSpeed, rightSpeed);
  setMotorSpeeds(leftSpeed, rightSpeed);
}

// ==================== INTERSECTION HANDLING (PLEDGE ALGORITHM)
// ====================
void handleIntersection(bool leftAvail, bool rightAvail) {
  // Drive forward past the intersection center
  driveForwardMs(INTERSECTION_FORWARD_MS);

  // Re-read sensors to check if straight path continues (include D6)
  readSensors();
  bool straightAvail = (s2 == 1 || s3 == 1 || s4 == 1 || s6 == 1);

  // Decide which way to turn
  char turn;

  if (runMode == RUN_EXPLORE) {
    // ===== PLEDGE ALGORITHM DECISION =====
    turn = pledgeDecide(leftAvail, straightAvail, rightAvail);

    // Record angle counter state and update
    int angleBefore = pledgeAngleCounter;
    updatePledgeAngle(turn);
    recordTurn(turn);
    angleAtTurn[pathIndex - 1] = angleBefore;
    angleAfterTurn[pathIndex - 1] = pledgeAngleCounter;

    Serial.print("Intersection: L=");
    Serial.print(leftAvail);
    Serial.print(" S=");
    Serial.print(straightAvail);
    Serial.print(" R=");
    Serial.print(rightAvail);
    Serial.print(" -> ");
    Serial.print(turn);
    Serial.print(" | AngleCtr=");
    Serial.print(pledgeAngleCounter);
    Serial.print(" WallFollow=");
    Serial.println(pledgeWallFollowing ? "YES" : "NO");

  } else {
    // RUN_SOLVED: follow pre-computed optimized path
    if (solvedPathIndex < pathIndex) {
      turn = path[solvedPathIndex];
      solvedPathIndex++;
      Serial.print("Solved turn #");
      Serial.print(solvedPathIndex);
      Serial.print(": ");
      Serial.println(turn);
    } else {
      turn = 'S';
    }
  }

  // Execute the turn
  executeTurn(turn);
}

// ==================== PLEDGE ALGORITHM DECISION ====================
char pledgeDecide(bool leftAvail, bool straightAvail, bool rightAvail) {
  if (!pledgeWallFollowing) {
    // Not wall-following: prefer straight (maintain original heading)
    if (straightAvail) {
      return 'S';
    }
    // Can't go straight → enter wall-following mode
    pledgeWallFollowing = true;
    Serial.println("Pledge: Cannot go straight, entering wall-follow mode");
  }

  // Wall-following mode: left-hand rule (L > S > R > B)
  if (leftAvail) {
    return 'L';
  } else if (straightAvail) {
    return 'S';
  } else if (rightAvail) {
    return 'R';
  } else {
    return 'B';
  }
}

// ==================== PLEDGE ANGLE COUNTER ====================
void updatePledgeAngle(char turn) {
  switch (turn) {
  case 'L':
    pledgeAngleCounter -= 90;
    break;
  case 'R':
    pledgeAngleCounter += 90;
    break;
  case 'B':
    pledgeAngleCounter += 180;
    break;
  case 'S':
    // No angle change for going straight
    break;
  }

  // Normalize to [-360, 360] range
  while (pledgeAngleCounter > 360)
    pledgeAngleCounter -= 360;
  while (pledgeAngleCounter < -360)
    pledgeAngleCounter += 360;

  // Key Pledge rule: if angle counter returns to 0, exit wall-following
  if (pledgeAngleCounter == 0 && pledgeWallFollowing) {
    pledgeWallFollowing = false;
    Serial.println("Pledge: Angle counter = 0, exiting wall-follow mode");
    // This marks the end of a dead branch → trigger angle-based optimization
    optimizePathByAngle();
  }
}

// ==================== DEAD END HANDLING ====================
void handleDeadEnd() {
  Serial.println("Dead end! U-turn");

  stopMotors();
  delay(DEAD_END_BRAKE_MS);

  if (runMode == RUN_EXPLORE) {
    int angleBefore = pledgeAngleCounter;
    updatePledgeAngle('B');
    recordTurn('B');
    angleAtTurn[pathIndex - 1] = angleBefore;
    angleAfterTurn[pathIndex - 1] = pledgeAngleCounter;

    Serial.print("Angle counter after U-turn: ");
    Serial.println(pledgeAngleCounter);
  }

  // Execute U-turn
  executeTurn('B');
}

// ==================== FINISH DETECTION ====================
void handleFinish() {
  stopMotors();
  robotRunning = false;
  allBlackStartMs = 0;

  if (runMode == RUN_EXPLORE) {
    mazeFinished = false;
    Serial.println("=============================");
    Serial.println("  MAZE SOLVED! (Run 1 done)");
    Serial.println("=============================");
    Serial.print("Raw path: ");
    printPath();
    Serial.print("Pledge angle counter: ");
    Serial.println(pledgeAngleCounter);

    // Optimize the path
    optimizePath();

    Serial.print("Optimized path: ");
    printPath();
    Serial.print("Path length: ");
    Serial.println(pathIndex);

    // Switch to solved mode (manual START command triggers run 2)
    runMode = RUN_SOLVED;
    solvedPathIndex = 0;
    lastError = 0;
    integral = 0;

    // Re-seed gyro + PID timestamps for Run 2
    lastGyroMicros = micros();
    lastPidMicros = micros();

    sendBoth("Run 2 prepared. Send START/S to run optimized path.");
  } else {
    mazeFinished = true;
    Serial.println("=============================");
    Serial.println("  RUN 2 COMPLETE! Optimized!");
    Serial.println("=============================");
    Serial.print("Final path: ");
    printPath();
    sendBoth("Maze complete. Send RESET/R for a new explore run.");
    // Stay stopped
  }
}

// ==================== TURN EXECUTION ====================
void executeTurn(char turn) {
  switch (turn) {
  case 'L':
    turnLeft90();
    break;
  case 'R':
    turnRight90();
    break;
  case 'S':
    // Go straight - small nudge forward
    driveForwardMs(30);
    break;
  case 'B':
    uTurn();
    break;
  }

  // After turning, reset PID state
  lastError = 0;
  integral = 0;
}

// ==================== PATH RECORDING & OPTIMIZATION ======================
void recordTurn(char turn) {
  if (pathIndex < MAX_PATH) {
    path[pathIndex] = turn;
    pathIndex++;

    // Try to optimize after each 'B' (dead-end u-turn)
    if (turn == 'B') {
      optimizePath();
    }
  }
}

// Angle-based dead branch elimination (Pledge)
// When pledgeAngleCounter returns to 0, the segment from the last
// zero-crossing to now is a dead branch loop that can be removed
void optimizePathByAngle() {
  if (pathIndex < 3)
    return;

  // Find the most recent point where angleCounter was 0 before the turn
  int branchStart = -1;
  for (int i = pathIndex - 2; i >= 0; i--) {
    if (angleAtTurn[i] == 0) {
      branchStart = i;
      break;
    }
  }

  if (branchStart >= 0 && branchStart < pathIndex - 1) {
    int removeCount = pathIndex - branchStart;
    Serial.print("Pledge: Removing dead branch of ");
    Serial.print(removeCount);
    Serial.println(" turns (angle returned to 0)");

    pathIndex = branchStart;
    for (int i = branchStart; i < branchStart + removeCount && i < MAX_PATH;
         i++) {
      path[i] = 0;
      angleAtTurn[i] = 0;
      angleAfterTurn[i] = 0;
    }
  }
}

// Traditional x-B-y optimization rules to eliminate dead ends
void optimizePath() {
  bool changed = true;
  while (changed) {
    changed = false;

    for (int i = 0; i < pathIndex - 2; i++) {
      if (path[i + 1] != 'B')
        continue; // Middle must be 'B'

      char a = path[i];
      char c = path[i + 2];
      char replacement = 0;

      if (a == 'L' && c == 'R')
        replacement = 'B';
      else if (a == 'L' && c == 'S')
        replacement = 'R';
      else if (a == 'L' && c == 'L')
        replacement = 'S';
      else if (a == 'S' && c == 'L')
        replacement = 'R';
      else if (a == 'S' && c == 'S')
        replacement = 'B';
      else if (a == 'R' && c == 'L')
        replacement = 'B';

      if (replacement != 0) {
        // Replace 3 entries with 1
        path[i] = replacement;
        // Shift remaining path left by 2
        for (int j = i + 1; j < pathIndex - 2; j++) {
          path[j] = path[j + 2];
        }
        pathIndex -= 2;
        path[pathIndex] = 0;
        path[pathIndex + 1] = 0;
        changed = true;
        break; // Restart scan
      }
    }
  }
}

void printPath() {
  for (int i = 0; i < pathIndex; i++) {
    Serial.print(path[i]);
    if (i < pathIndex - 1)
      Serial.print("-");
  }
  Serial.println();
}

// ==================== HELPER: DRIVE FORWARD ====================
void driveForwardMs(int ms) {
  setMotorSpeeds(FORWARD_NUDGE, FORWARD_NUDGE);
  delay(ms);
}

// ==================== PID CONTROL ====================
int calculateError() {
  // D6 has weight 0 — adds to denominator only, pulling error toward center
  int weightedSum = (s1 * WEIGHT_D1) + (s2 * WEIGHT_D2) + (s3 * WEIGHT_D3) +
                    (s4 * WEIGHT_D4) + (s5 * WEIGHT_D5);
  int activeCount = s1 + s2 + s3 + s4 + s5 + s6;

  if (activeCount == 0) {
    integral = 0;
    return (lastError > 0) ? 5 : -5;
  }

  return weightedSum / activeCount;
}

int pidControl(int error, float dtRatio) {
  // dtRatio = actualDt / expectedDt (1.0 at normal 2ms rate)
  // Gains (KP, KI, KD) stay tuned for 2ms intervals;
  // dtRatio compensates when an iteration is late or early.
  float P = KP * error;

  integral += error * dtRatio; // Time-scaled accumulation
  integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  float I = KI * integral;

  float D = (dtRatio > 0.01f) ? KD * (error - lastError) / dtRatio : 0;
  lastError = error;

  return (int)(P + I + D);
}

// ==================== MOTOR CONTROL ====================
void applyMinSpeed(int &left, int &right) {
  if (left > 0 && left < MIN_SPEED)
    left = MIN_SPEED;
  if (left < 0 && left > -MIN_SPEED)
    left = -MIN_SPEED;
  if (right > 0 && right < MIN_SPEED)
    right = MIN_SPEED;
  if (right < 0 && right > -MIN_SPEED)
    right = -MIN_SPEED;
}

void setMotorSpeeds(int left, int right) {
  // Left motor
  if (left >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
    ledcWrite(MOTOR_LEFT_PWM, left);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
    ledcWrite(MOTOR_LEFT_PWM, -left);
  }

  // Right motor
  if (right >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
    ledcWrite(MOTOR_RIGHT_PWM, right);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
    ledcWrite(MOTOR_RIGHT_PWM, -right);
  }
}

void stopMotors() {
  ledcWrite(MOTOR_LEFT_PWM, 0);
  ledcWrite(MOTOR_RIGHT_PWM, 0);
}
