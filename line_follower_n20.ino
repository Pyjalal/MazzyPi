/*
 * Line Following Robot with PID Control
 * Using Maker Line (5 sensors) & Maker Robo ESP32
 * Tuned for N20 Motors (600RPM, 30:1 gear ratio, 6V)
 *
 * N20 Motor Specs:
 *   Rated voltage: 6VDC (3V-9V range)
 *   Free run speed @ 6V: 600 RPM
 *   Free run current @ 6V: 50mA
 *   Rated torque: 0.065 kg-cm @ 500 RPM, 150mA
 *   Stall torque @ 6V: 0.3 kg-cm, 500mA
 *   Gear ratio: 30:1
 *
 * Maker Line Sensor Pins:
 *   D1 (Leftmost):  32
 *   D2 (Left):      39
 *   D3 (Center):    21
 *   D4 (Right):     22
 *   D5 (Rightmost): 25
 *
 * PID Control for smooth line following
 */

// ==================== PIN DEFINITIONS ====================
// Maker Line Sensor Pins (5 sensors)
#define SENSOR_D1 32 // Leftmost
#define SENSOR_D2 39 // Left
#define SENSOR_D3 21 // Center
#define SENSOR_D4 22 // Right
#define SENSOR_D5 25 // Rightmost

// Motor Driver Pins (Maker Robo ESP32)
#define MOTOR_LEFT_PWM 27
#define MOTOR_LEFT_DIR 14
#define MOTOR_RIGHT_PWM 13
#define MOTOR_RIGHT_DIR 12

// LEDC PWM Configuration
#define PWM_FREQ 20000    // 20kHz - above audible range for quiet operation
#define PWM_RESOLUTION 10 // 10-bit resolution (0-1023)
#define PWM_MAX 1023      // Maximum PWM value for 10-bit

// ==================== PID TUNING PARAMETERS ====================
// N20 motors are ~3x faster and more responsive than TT motors
// Lower gains needed to prevent oscillation and overshoot
#define KP                                                                     \
  10.0 // Proportional gain (research: ~10-15 for digital ±5 error range)
#define KI 0.0 // Integral gain (research: 0 for fast N20 - PD only is standard)
#define KD 12.0 // Derivative gain (research: ~1-2x Kp for digital sensors)

// Integral windup limit
#define INTEGRAL_MAX 200 // Prevents integral term from accumulating too much

// ==================== SPEED SETTINGS ====================
// N20 @ 600RPM is ~3x faster than TT @ ~200RPM, so scale down PWM values
#define BASE_SPEED 200 // Normal cruising speed
#define MAX_SPEED 400  // Maximum motor speed (was 720 for TT)
#define MIN_SPEED 80 // Minimum speed - N20 has lower deadband than TT (was 160)
#define SEARCH_SPEED 160     // Max speed when searching for lost line
#define SEARCH_SPEED_MIN 100 // Initial slow search speed to avoid overshoot

// ==================== SENSOR WEIGHTS ====================
// Weights for error calculation: leftmost = -4, rightmost = +4
const int WEIGHT_D1 = -4; // Leftmost
const int WEIGHT_D2 = -2; // Left
const int WEIGHT_D3 = 0;  // Center
const int WEIGHT_D4 = 2;  // Right
const int WEIGHT_D5 = 4;  // Rightmost

// ==================== VARIABLES ====================
int s1, s2, s3, s4, s5;            // Sensor readings
float lastError = 0;               // For derivative calculation
float integral = 0;                // For integral calculation
int searchDirection = 1;           // 1 = search right, -1 = search left
unsigned long searchStartTime = 0; // When search mode began
bool wasSearching = false;         // Track if we were searching last loop

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println("Line Follower - 5 Sensor PID Control");
  Serial.println("N20 Motors (600RPM 30:1) + Maker Robo ESP32");

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

  Serial.println("Starting in 2 seconds...");
  delay(2000);
  Serial.println("GO!");
}

// ==================== MAIN LOOP ====================
void loop() {
  readSensors();

  int error = calculateError();
  int correction = pidControl(error);

  applyMotorSpeeds(correction);

  // Debug output (comment out for faster loop)
  // printDebug(error, correction);

  delay(2); // Shorter delay - N20 responds faster than TT motors
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
    // Line lost - return large error based on last known direction
    // Reset integral to avoid windup during line loss
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
  float P = KP * error;

  // Integral term (accumulated error)
  integral += error;
  integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  float I = KI * integral;

  // Derivative term (rate of change)
  float D = KD * (error - lastError);

  // Store current error for next iteration
  lastError = error;

  // PID output
  return (int)(P + I + D);
}

// ==================== MOTOR CONTROL ====================
void applyMotorSpeeds(int correction) {
  int leftSpeed = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  // Check if line is lost (all sensors white)
  if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0) {
    if (!wasSearching) {
      // Just lost the line - brake briefly to kill momentum
      searchStartTime = millis();
      wasSearching = true;
      leftSpeed = 0;
      rightSpeed = 0;
    } else {
      // Progressive search: start slow, ramp up over 500ms
      unsigned long elapsed = millis() - searchStartTime;
      int speed = SEARCH_SPEED_MIN;
      if (elapsed > 50) { // Brief brake for first 50ms
        speed = map(constrain(elapsed, 50, 500), 50, 500, SEARCH_SPEED_MIN,
                    SEARCH_SPEED);
      } else {
        speed = 0; // Still braking
      }
      // Gentle arc: one wheel forward, other stopped (not reversed)
      if (searchDirection > 0) {
        leftSpeed = speed;
        rightSpeed = 0;
      } else {
        leftSpeed = 0;
        rightSpeed = speed;
      }
    }
  } else {
    // Line found - reset search state
    wasSearching = false;
  }

  // Constrain speeds
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  // Apply minimum speed threshold (lower for N20 - less friction/deadband)
  if (leftSpeed > 0 && leftSpeed < MIN_SPEED)
    leftSpeed = MIN_SPEED;
  if (leftSpeed < 0 && leftSpeed > -MIN_SPEED)
    leftSpeed = -MIN_SPEED;
  if (rightSpeed > 0 && rightSpeed < MIN_SPEED)
    rightSpeed = MIN_SPEED;
  if (rightSpeed < 0 && rightSpeed > -MIN_SPEED)
    rightSpeed = -MIN_SPEED;

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

// ==================== DEBUG ====================
void printDebug(int error, int correction) {
  Serial.print("S:");
  Serial.print(s1);
  Serial.print(s2);
  Serial.print(s3);
  Serial.print(s4);
  Serial.print(s5);
  Serial.print(" E:");
  Serial.print(error);
  Serial.print(" I:");
  Serial.print(integral, 1);
  Serial.print(" C:");
  Serial.println(correction);
}
