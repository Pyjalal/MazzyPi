/*
 * Line Following Robot with PD Control
 * Using Maker Line (5 sensors) & Maker Robo ESP32
 *
 * Maker Line Sensor Pins:
 *   D1 (Leftmost):  32
 *   D2 (Left):      39
 *   D3 (Center):    21
 *   D4 (Right):     22
 *   D5 (Rightmost): 25
 *
 * PD Control for smooth line following
 */

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
#define PWM_FREQ 20000    // 20kHz - above audible range for quiet operation
#define PWM_RESOLUTION 10 // 10-bit resolution (0-1023)
#define PWM_MAX 1023      // Maximum PWM value for 10-bit

// ==================== PD TUNING PARAMETERS ====================
#define KP 25 // Proportional gain - increase for sharper response
#define KD 15 // Derivative gain - increase to reduce oscillation

// ==================== SPEED SETTINGS ====================
#define BASE_SPEED 320   // Normal cruising speed (0-1023 for 10-bit)
#define MAX_SPEED 720    // Maximum motor speed
#define MIN_SPEED 160    // Minimum speed to overcome motor deadband
#define SEARCH_SPEED 360 // Speed when searching for lost line

// ==================== SENSOR WEIGHTS ====================
// Weights for error calculation: leftmost = -4, rightmost = +4
const int WEIGHT_D1 = -4; // Leftmost
const int WEIGHT_D2 = -2; // Left
const int WEIGHT_D3 = 0;  // Center
const int WEIGHT_D4 = 2;  // Right
const int WEIGHT_D5 = 4;  // Rightmost

// ==================== VARIABLES ====================
int s1, s2, s3, s4, s5;  // Sensor readings
int lastError = 0;       // For derivative calculation
int searchDirection = 1; // 1 = search right, -1 = search left

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println("Line Follower - 5 Sensor PD Control");
  Serial.println("Maker Line + Maker Robo ESP32");

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
  int correction = pdControl(error);

  applyMotorSpeeds(correction);

  // Debug output (comment out for faster loop)
  // printDebug(error, correction);

  delay(5); // Small delay for stability
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

// ==================== PD CONTROL ====================
int pdControl(int error) {
  // Proportional term
  int P = KP * error;

  // Derivative term (rate of change)
  int D = KD * (error - lastError);

  // Store current error for next iteration
  lastError = error;

  // PD output
  return P + D;
}

// ==================== MOTOR CONTROL ====================
void applyMotorSpeeds(int correction) {
  int leftSpeed = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  // Check if line is lost (all sensors white)
  if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0) {
    // Search mode - spin toward last known direction
    if (searchDirection > 0) {
      leftSpeed = SEARCH_SPEED;
      rightSpeed = -SEARCH_SPEED / 2;
    } else {
      leftSpeed = -SEARCH_SPEED / 2;
      rightSpeed = SEARCH_SPEED;
    }
  }

  // Constrain speeds
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  // Apply minimum speed threshold
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
  Serial.print(" C:");
  Serial.println(correction);
}
