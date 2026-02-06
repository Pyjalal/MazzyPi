/*
 * Line Following Robot with Bluetooth PID Calibration
 * Using Maker Line (5 sensors) & Maker Robo ESP32
 * Tuned for N20 Motors (600RPM, 30:1 gear ratio, 6V)
 *
 * N20 Motor Specs:
 *   Rated voltage: 6VDC (3V-9V range)
 *   Free run speed @ 6V: 600 RPM
 *   Rated torque: 0.065 kg-cm @ 500 RPM
 *   Stall torque @ 6V: 0.3 kg-cm
 *   Gear ratio: 30:1
 *
 * Bluetooth Commands (via Serial Bluetooth Terminal app):
 *   P<value>   - Set KP         e.g. P12.5
 *   I<value>   - Set KI         e.g. I0.8
 *   D<value>   - Set KD         e.g. D8.0
 *   B<value>   - Set base speed e.g. B200
 *   M<value>   - Set max speed  e.g. M400
 *   N<value>   - Set min speed  e.g. N80
 *   S<value>   - Set search spd e.g. S200
 *   W<value>   - Set integ max  e.g. W200
 *   GO         - Start following
 *   STOP       - Stop motors
 *   GET        - Print all current values
 *   TEL        - Toggle live telemetry on/off
 *   RESET      - Reset integral accumulator
 *   HELP       - Show command list
 *
 * Connect via "Serial Bluetooth Terminal" app on Android
 * Device name: "LineFollower_N20"
 */

#include "BluetoothSerial.h"

BluetoothSerial BT;

// ==================== PIN DEFINITIONS ====================
// Maker Line Sensor Pins (5 sensors)
#define SENSOR_D1 32 // Leftmost
#define SENSOR_D2 39 // Left
#define SENSOR_D3 21 // Center
#define SENSOR_D4 22 // Right
#define SENSOR_D5 25 // Rightmost

// Motor Driver Pins (Maker Robo ESP32)
#define MOTOR_LEFT_PWM 13
#define MOTOR_LEFT_DIR 12
#define MOTOR_RIGHT_PWM 27
#define MOTOR_RIGHT_DIR 14

// LEDC PWM Configuration
#define PWM_FREQ 20000    // 20kHz - above audible range
#define PWM_RESOLUTION 10 // 10-bit resolution (0-1023)
#define PWM_MAX 1023      // Maximum PWM value for 10-bit

// ==================== PID TUNING PARAMETERS (runtime adjustable) ===========
float kp = 12.0;  // Proportional gain
float ki = 0.5;   // Integral gain
float kd = 8.0;   // Derivative gain
float integralMax = 200.0; // Integral windup limit

// ==================== SPEED SETTINGS (runtime adjustable) ==================
int baseSpeed   = 180;  // Normal cruising speed
int maxSpeed    = 400;  // Maximum motor speed
int minSpeed    = 80;   // Minimum speed (deadband)
int searchSpeed = 200;  // Speed when searching for lost line

// ==================== SENSOR WEIGHTS ====================
const int WEIGHT_D1 = -4; // Leftmost
const int WEIGHT_D2 = -2; // Left
const int WEIGHT_D3 = 0;  // Center
const int WEIGHT_D4 = 2;  // Right
const int WEIGHT_D5 = 4;  // Rightmost

// ==================== VARIABLES ====================
int s1, s2, s3, s4, s5;     // Sensor readings
float lastError = 0;        // For derivative calculation
float integral = 0;         // For integral calculation
int searchDirection = 1;    // 1 = search right, -1 = search left

bool running = false;       // Motor enable flag (controlled via BT)
bool telemetryOn = false;   // Live telemetry toggle
unsigned long lastTelemetry = 0;
#define TELEMETRY_INTERVAL 100 // ms between telemetry prints

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);

  // Initialize Bluetooth
  BT.begin("LineFollower_N20");
  Serial.println("Bluetooth started: LineFollower_N20");

  // Initialize sensor pins
  pinMode(SENSOR_D1, INPUT);
  pinMode(SENSOR_D2, INPUT);
  pinMode(SENSOR_D3, INPUT);
  pinMode(SENSOR_D4, INPUT);
  pinMode(SENSOR_D5, INPUT);

  // Initialize motor direction pins
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);

  // Setup LEDC for PWM motor control
  ledcAttach(MOTOR_LEFT_PWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_RIGHT_PWM, PWM_FREQ, PWM_RESOLUTION);

  // Stop motors initially
  stopMotors();

  btPrint("=== LineFollower N20 BT Calibration ===");
  btPrint("Type HELP for commands");
  btPrint("Type GO to start, STOP to halt");
  printValues();
}

// ==================== MAIN LOOP ====================
void loop() {
  // Handle Bluetooth commands
  handleBluetooth();

  // Only run PID line following when enabled
  if (running) {
    readSensors();

    int error = calculateError();
    int correction = pidControl(error);

    applyMotorSpeeds(correction);

    // Live telemetry via Bluetooth
    if (telemetryOn && (millis() - lastTelemetry >= TELEMETRY_INTERVAL)) {
      lastTelemetry = millis();
      sendTelemetry(error, correction);
    }

    delay(2); // Short delay for N20 fast response
  }
}

// ==================== BLUETOOTH HANDLER ====================
void handleBluetooth() {
  if (!BT.available()) return;

  String cmd = BT.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  // Also echo to USB serial for debugging
  Serial.print("BT> ");
  Serial.println(cmd);

  char first = cmd.charAt(0);
  String valueStr = cmd.substring(1);
  float value = valueStr.toFloat();

  if (cmd.equalsIgnoreCase("GO")) {
    running = true;
    integral = 0;
    lastError = 0;
    btPrint(">> STARTED - Motors ON");
  }
  else if (cmd.equalsIgnoreCase("STOP")) {
    running = false;
    stopMotors();
    btPrint(">> STOPPED - Motors OFF");
  }
  else if (cmd.equalsIgnoreCase("GET")) {
    printValues();
  }
  else if (cmd.equalsIgnoreCase("TEL")) {
    telemetryOn = !telemetryOn;
    btPrint(telemetryOn ? ">> Telemetry ON" : ">> Telemetry OFF");
  }
  else if (cmd.equalsIgnoreCase("RESET")) {
    integral = 0;
    lastError = 0;
    btPrint(">> PID state reset");
  }
  else if (cmd.equalsIgnoreCase("HELP")) {
    printHelp();
  }
  else if (first == 'P' || first == 'p') {
    kp = value;
    btPrint(">> KP = " + String(kp, 2));
  }
  else if (first == 'I' || first == 'i') {
    ki = value;
    btPrint(">> KI = " + String(ki, 2));
  }
  else if (first == 'D' || first == 'd') {
    kd = value;
    btPrint(">> KD = " + String(kd, 2));
  }
  else if (first == 'B' || first == 'b') {
    baseSpeed = (int)value;
    btPrint(">> BASE_SPEED = " + String(baseSpeed));
  }
  else if (first == 'M' || first == 'm') {
    maxSpeed = (int)value;
    btPrint(">> MAX_SPEED = " + String(maxSpeed));
  }
  else if (first == 'N' || first == 'n') {
    minSpeed = (int)value;
    btPrint(">> MIN_SPEED = " + String(minSpeed));
  }
  else if (first == 'S' || first == 's') {
    searchSpeed = (int)value;
    btPrint(">> SEARCH_SPEED = " + String(searchSpeed));
  }
  else if (first == 'W' || first == 'w') {
    integralMax = value;
    btPrint(">> INTEGRAL_MAX = " + String(integralMax, 1));
  }
  else {
    btPrint("?? Unknown: " + cmd);
    btPrint("Type HELP for commands");
  }
}

// ==================== BLUETOOTH HELPERS ====================
void btPrint(String msg) {
  BT.println(msg);
  Serial.println(msg);
}

void printValues() {
  btPrint("--- Current PID Values ---");
  btPrint("  KP = " + String(kp, 2));
  btPrint("  KI = " + String(ki, 2));
  btPrint("  KD = " + String(kd, 2));
  btPrint("  INTEGRAL_MAX = " + String(integralMax, 1));
  btPrint("--- Speed Settings ---");
  btPrint("  BASE  = " + String(baseSpeed));
  btPrint("  MAX   = " + String(maxSpeed));
  btPrint("  MIN   = " + String(minSpeed));
  btPrint("  SEARCH= " + String(searchSpeed));
  btPrint("--- State ---");
  btPrint("  Running: " + String(running ? "YES" : "NO"));
  btPrint("  Telemetry: " + String(telemetryOn ? "ON" : "OFF"));
  btPrint("--------------------------");
}

void printHelp() {
  btPrint("=== COMMANDS ===");
  btPrint("P<val> - Set KP      (e.g. P12.5)");
  btPrint("I<val> - Set KI      (e.g. I0.8)");
  btPrint("D<val> - Set KD      (e.g. D8.0)");
  btPrint("B<val> - Base speed  (e.g. B200)");
  btPrint("M<val> - Max speed   (e.g. M400)");
  btPrint("N<val> - Min speed   (e.g. N80)");
  btPrint("S<val> - Search spd  (e.g. S200)");
  btPrint("W<val> - Integ limit (e.g. W200)");
  btPrint("GO     - Start motors");
  btPrint("STOP   - Stop motors");
  btPrint("GET    - Show all values");
  btPrint("TEL    - Toggle telemetry");
  btPrint("RESET  - Reset PID state");
  btPrint("HELP   - This message");
  btPrint("================");
}

void sendTelemetry(int error, int correction) {
  String msg = "S:" + String(s1) + String(s2) + String(s3) + String(s4) + String(s5)
             + " E:" + String(error)
             + " I:" + String(integral, 1)
             + " C:" + String(correction);
  BT.println(msg);
}

// ==================== SENSOR FUNCTIONS ====================
void readSensors() {
  // Sensors return 1 for black line (inverted logic)
  s1 = digitalRead(SENSOR_D1); // Leftmost
  s2 = digitalRead(SENSOR_D2); // Left
  s3 = digitalRead(SENSOR_D3); // Center
  s4 = digitalRead(SENSOR_D4); // Right
  s5 = digitalRead(SENSOR_D5); // Rightmost
}

int calculateError() {
  // Calculate weighted sum
  int weightedSum = (s1 * WEIGHT_D1) + (s2 * WEIGHT_D2) + (s3 * WEIGHT_D3) +
                    (s4 * WEIGHT_D4) + (s5 * WEIGHT_D5);

  // Count active sensors
  int activeCount = s1 + s2 + s3 + s4 + s5;

  if (activeCount == 0) {
    // Line lost - reset integral to avoid windup
    integral = 0;
    return (lastError > 0) ? 5 : -5;
  }

  // Average error (normalized)
  int error = weightedSum / activeCount;

  // Remember search direction for recovery
  if (error < 0)
    searchDirection = -1;
  else if (error > 0)
    searchDirection = 1;

  return error;
}

// ==================== PID CONTROL ====================
int pidControl(int error) {
  // Proportional term
  float P = kp * error;

  // Integral term (accumulated error)
  integral += error;
  integral = constrain(integral, -integralMax, integralMax);
  float I = ki * integral;

  // Derivative term (rate of change)
  float D = kd * (error - lastError);

  // Store current error for next iteration
  lastError = error;

  // PID output
  return (int)(P + I + D);
}

// ==================== MOTOR CONTROL ====================
void applyMotorSpeeds(int correction) {
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  // Check if line is lost (all sensors white)
  if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0) {
    // Search mode - spin toward last known direction
    if (searchDirection > 0) {
      leftSpeed = searchSpeed;
      rightSpeed = -searchSpeed / 2;
    } else {
      leftSpeed = -searchSpeed / 2;
      rightSpeed = searchSpeed;
    }
  }

  // Constrain speeds
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  // Apply minimum speed threshold (lower for N20 - less friction/deadband)
  if (leftSpeed > 0 && leftSpeed < minSpeed)
    leftSpeed = minSpeed;
  if (leftSpeed < 0 && leftSpeed > -minSpeed)
    leftSpeed = -minSpeed;
  if (rightSpeed > 0 && rightSpeed < minSpeed)
    rightSpeed = minSpeed;
  if (rightSpeed < 0 && rightSpeed > -minSpeed)
    rightSpeed = -minSpeed;

  setMotorSpeeds(leftSpeed, rightSpeed);
}

void setMotorSpeeds(int left, int right) {
  // Left motor
  if (left >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, LOW); // Forward
    ledcWrite(MOTOR_LEFT_PWM, left);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, HIGH); // Backward
    ledcWrite(MOTOR_LEFT_PWM, -left);
  }

  // Right motor
  if (right >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, LOW); // Forward
    ledcWrite(MOTOR_RIGHT_PWM, right);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH); // Backward
    ledcWrite(MOTOR_RIGHT_PWM, -right);
  }
}

void stopMotors() {
  ledcWrite(MOTOR_LEFT_PWM, 0);
  ledcWrite(MOTOR_RIGHT_PWM, 0);
}
