/*
 * Relay Auto-Tune for Line Follower
 * ESP32 + Maker Line (5 sensors) + N20 Motors
 *
 * This uses the Astrom-Hagglund Relay Method (Ziegler-Nichols).
 * It drives forward at a base speed, but steers using Bang-Bang (relay) control:
 *   - If error > 0, steer hard right (+relayAmplitude)
 *   - If error < 0, steer hard left (-relayAmplitude)
 * 
 * By measuring the amplitude of the resulting oscillation (error peak)
 * and the period of the oscillation (time between crossings),
 * it calculates Ku (Ultimate Gain) and Tu (Ultimate Period).
 * It then applies ZN rules (specifically PD rules for line followers)
 * to set Kp and Kd.
 *
 * BT COMMANDS:
 *   AT        Start Relay Auto-tune
 *   S/START   Start normal PID following with tuned gains
 *   X/STOP    Stop motors
 *   FLIP      Toggle steering polarity
 *   SPD xx    Set speed
 *   BEST      Show current PID values
 */

#include <Arduino.h>
#include <math.h>
#include "BluetoothSerial.h"
#include <Preferences.h>

BluetoothSerial SerialBT;
Preferences prefs;
String btBuffer = "";

// ==================== PINS ====================
#define SENSOR_D1 32
#define SENSOR_D2 39
#define SENSOR_D3 21
#define SENSOR_D4 22
#define SENSOR_D5 25
#define MOTOR_LEFT_PWM  14
#define MOTOR_LEFT_DIR  27
#define MOTOR_RIGHT_PWM 13
#define MOTOR_RIGHT_DIR 12

// ==================== PWM ====================
#define PWM_FREQ       20000
#define PWM_RESOLUTION 10
#define PWM_MAX        ((1 << PWM_RESOLUTION) - 1)

// ==================== SENSOR WEIGHTS ====================
const int WEIGHT_D1=-4, WEIGHT_D2=-2, WEIGHT_D3=0, WEIGHT_D4=2, WEIGHT_D5=4;

// ==================== PID ====================
float kp = 10.0f;
float ki = 0.0f;
float kd = 4.0f;
float lastError = 0.0f;
float integral = 0.0f;
#define INTEGRAL_MAX 200.0f

// ==================== SPEED ====================
int baseSpeed = 200;
#define MAX_SPEED  PWM_MAX
#define MIN_SPEED  90
#define RAMP_STEP  12

// ==================== GLOBALS ====================
int s1,s2,s3,s4,s5;
int currentLeftSpeed=0, currentRightSpeed=0;
bool robotRunning = false;
bool invertSteering = false;

// ==================== RELAY AUTO-TUNE ====================
bool isTuning = false;
int relayAmplitude = 100;     // How hard to steer during oscillation
float tuneErrMax = 0.0;       // Peak error amplitude measured
float tuneErrMin = 0.0;
unsigned long lastCrossTime = 0;
unsigned long tuneStartTime = 0;
int crossCount = 0;
float periodSum = 0;
int relayState = 1;           // 1 for Right, -1 for Left

#define TUNE_MIN_CROSSINGS 6
#define TUNE_MAX_TIME_MS   10000

// ==================== FORWARD DECL ====================
void sendBoth(const String &msg);
void readSensors();
float calculateError();
int pidControl(float error);
int rampTo(int cur, int tgt, int step);
void setMotorSpeeds(int left, int right);
void stopMotors();
void handleBT();
void processCommand(String cmd);
void startRelayTune();
void relayStep(float error);
void finishRelayTune();
void saveGainsToNVS();
void loadGainsFromNVS();

// ==================== NVS ====================
void saveGainsToNVS() {
  prefs.begin("relaytune", false);
  prefs.putFloat("kp", kp);
  prefs.putFloat("kd", kd);
  prefs.putInt("spd", baseSpeed);
  prefs.end();
}
void loadGainsFromNVS() {
  prefs.begin("relaytune", true);
  kp = prefs.getFloat("kp", 10.0f);
  kd = prefs.getFloat("kd", 4.0f);
  baseSpeed = prefs.getInt("spd", 200);
  prefs.end();
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("RelayTuner");
  
  pinMode(SENSOR_D1,INPUT); pinMode(SENSOR_D2,INPUT); pinMode(SENSOR_D3,INPUT);
  pinMode(SENSOR_D4,INPUT); pinMode(SENSOR_D5,INPUT);
  pinMode(MOTOR_LEFT_DIR,OUTPUT); pinMode(MOTOR_RIGHT_DIR,OUTPUT);
  ledcAttach(MOTOR_LEFT_PWM,PWM_FREQ,PWM_RESOLUTION);
  ledcAttach(MOTOR_RIGHT_PWM,PWM_FREQ,PWM_RESOLUTION);
  stopMotors();
  
  loadGainsFromNVS();
  
  sendBoth("=== Relay Auto-Tuner ===");
  sendBoth("BT: RelayTuner");
  sendBoth("Loaded: spd="+String(baseSpeed)+" Kp="+String(kp,2)+" Kd="+String(kd,2));
  sendBoth("Place on line, send AT to oscillate and tune.");
}

// ==================== MAIN LOOP ====================
void loop() {
  handleBT();
  if(!robotRunning){ stopMotors(); delay(10); return; }
  
  readSensors();
  float rawErr = calculateError();
  
  // Abort if line lost completely
  if(s1==0 && s2==0 && s3==0 && s4==0 && s5==0) {
    if (isTuning) {
      sendBoth("Line lost during tuning! Aborting.");
      isTuning = false;
    }
    robotRunning = false;
    stopMotors();
    sendBoth("STOPPED (Line lost)");
    delay(500);
    return;
  }
  
  if (isTuning) {
    relayStep(rawErr);
  } else {
    // Normal PID
    int corr = pidControl(rawErr);
    int lT = baseSpeed + corr;
    int rT = baseSpeed - corr;
    lT = constrain(lT, -MAX_SPEED, MAX_SPEED);
    rT = constrain(rT, -MAX_SPEED, MAX_SPEED);
    if(lT>0 && lT<MIN_SPEED) lT=MIN_SPEED;
    if(rT>0 && rT<MIN_SPEED) rT=MIN_SPEED;
    
    currentLeftSpeed = rampTo(currentLeftSpeed, lT, RAMP_STEP);
    currentRightSpeed = rampTo(currentRightSpeed, rT, RAMP_STEP);
    setMotorSpeeds(currentLeftSpeed, currentRightSpeed);
  }
  delay(2);
}

// ==================== RELAY AUTO-TUNE ====================
void startRelayTune() {
  isTuning = true;
  robotRunning = true;
  tuneErrMax = 0.0;
  tuneErrMin = 0.0;
  crossCount = 0;
  periodSum = 0;
  relayState = 1;
  lastCrossTime = millis();
  tuneStartTime = millis();
  
  // Steer amplitude is roughly half the base speed
  relayAmplitude = baseSpeed / 2;
  if(relayAmplitude < 50) relayAmplitude = 50;
  
  sendBoth("=== RELAY TUNING STARTED ===");
  sendBoth("Speed: " + String(baseSpeed) + ", Relay Amp: " + String(relayAmplitude));
}

void relayStep(float error) {
  unsigned long now = millis();
  
  // Track peak error
  if(error > tuneErrMax) tuneErrMax = error;
  if(error < tuneErrMin) tuneErrMin = error;
  
  // Bang-Bang logic
  int nextState = relayState;
  if (error > 0.1f) nextState = 1;       // line is left -> steer left
  else if (error < -0.1f) nextState = -1; // line is right -> steer right
  
  // Detect zero crossing
  if (nextState != relayState) {
    unsigned long halfPeriod = now - lastCrossTime;
    
    // Ignore noise (bounces faster than 20ms)
    if (halfPeriod > 20) {
      crossCount++;
      if (crossCount > 1) { // Skip first partial crossing
        periodSum += (halfPeriod * 2.0f);
      }
      relayState = nextState;
      lastCrossTime = now;
      
      // Print progress
      if (crossCount % 2 == 0) {
        Serial.printf("Cross %d, P=%.0f ms, Amp=%.1f\n", crossCount, halfPeriod*2.0f, max(tuneErrMax, -tuneErrMin));
      }
    }
  }
  
  // Apply Relay Steering
  int steer = relayState * relayAmplitude;
  
  // Instead of turning in place, we want to drive forward WHILE oscillating
  int lT = baseSpeed + steer;
  int rT = baseSpeed - steer;
  
  lT = constrain(lT, -MAX_SPEED, MAX_SPEED);
  rT = constrain(rT, -MAX_SPEED, MAX_SPEED);
  
  // Instant snap for relay (no ramp)
  setMotorSpeeds(lT, rT);
  
  // Check completion
  if (crossCount >= TUNE_MIN_CROSSINGS) {
    finishRelayTune();
  } else if (now - tuneStartTime > TUNE_MAX_TIME_MS) {
    sendBoth("Tuning timed out!");
    isTuning = false;
    robotRunning = false;
    stopMotors();
  }
}

void finishRelayTune() {
  isTuning = false;
  robotRunning = false;
  stopMotors();
  
  // Calculate average ultimate period (Tu) in seconds
  float Tu = (periodSum / (crossCount - 1)) / 1000.0f;
  
  // Calculate average peak amplitude (a)
  float a = (tuneErrMax - tuneErrMin) / 2.0f;
  
  if (a < 0.1f) {
    sendBoth("Error: Amplitude too small. Check sensors.");
    return;
  }
  
  // Relay amplitude (d) is the steering effort applied
  float d = relayAmplitude;
  
  // Ultimate Gain (Ku) formula for relay method: 4 * d / (pi * a)
  float Ku = (4.0f * d) / (PI * a);
  
  sendBoth("\n=== RELAY RESULTS ===");
  sendBoth("Oscillation Period (Tu): " + String(Tu, 3) + " s");
  sendBoth("Error Amplitude (a): " + String(a, 2));
  sendBoth("Ultimate Gain (Ku): " + String(Ku, 2));
  
  // Ziegler-Nichols rules for PD controller (P and D only)
  // For Line Followers, integral is often zero to prevent windup on straightaways.
  // ZN PD Rules: Kp = 0.8 * Ku, Kd = Kp * Tu / 8
  
  kp = 0.8f * Ku;
  ki = 0.0f; 
  kd = kp * Tu / 8.0f;
  
  // Apply a damping factor since ZN is notoriously aggressive
  kp = kp * 0.6f;
  kd = kd * 1.5f; // extra damping for line follower stability
  
  sendBoth("\n=== TUNED ZN-PD GAINS (Damped) ===");
  sendBoth("Kp = " + String(kp, 2));
  sendBoth("Kd = " + String(kd, 2));
  
  saveGainsToNVS();
  sendBoth("Gains saved. Send S to test.");
}

// ==================== SENSOR & ERROR ====================
void readSensors(){ 
  s1=digitalRead(SENSOR_D1); s2=digitalRead(SENSOR_D2);
  s3=digitalRead(SENSOR_D3); s4=digitalRead(SENSOR_D4); s5=digitalRead(SENSOR_D5); 
}

float calculateError(){
  int ws=(s1*WEIGHT_D1)+(s2*WEIGHT_D2)+(s3*WEIGHT_D3)+(s4*WEIGHT_D4)+(s5*WEIGHT_D5);
  int ac=s1+s2+s3+s4+s5;
  if(invertSteering) ws=-ws;
  if(ac==0){ integral=0; return(lastError>0)?5.0f:-5.0f; }
  return (float)ws/(float)ac;
}

// ==================== PID ====================
int pidControl(float error){
  float P=kp*error;
  integral+=error;
  if(integral>INTEGRAL_MAX)integral=INTEGRAL_MAX;
  if(integral<-INTEGRAL_MAX)integral=-INTEGRAL_MAX;
  float D=kd*(error-lastError); lastError=error;
  return(int)(P+ki*integral+D);
}

// ==================== MOTOR ====================
int rampTo(int c,int t,int s){ if(c<t)return min(c+s,t); if(c>t)return max(c-s,t); return c; }
void setMotorSpeeds(int left,int right){
  left=constrain(left,-MAX_SPEED,MAX_SPEED); right=constrain(right,-MAX_SPEED,MAX_SPEED);
  if(left>=0){digitalWrite(MOTOR_LEFT_DIR,LOW);ledcWrite(MOTOR_LEFT_PWM,left);}
  else{digitalWrite(MOTOR_LEFT_DIR,HIGH);ledcWrite(MOTOR_LEFT_PWM,-left);}
  if(right>=0){digitalWrite(MOTOR_RIGHT_DIR,LOW);ledcWrite(MOTOR_RIGHT_PWM,right);}
  else{digitalWrite(MOTOR_RIGHT_DIR,HIGH);ledcWrite(MOTOR_RIGHT_PWM,-right);}
}
void stopMotors(){ 
  ledcWrite(MOTOR_LEFT_PWM,0); ledcWrite(MOTOR_RIGHT_PWM,0);
  currentLeftSpeed=0; currentRightSpeed=0; 
}
void sendBoth(const String &msg){ 
  Serial.println(msg);
  if(SerialBT.hasClient()) SerialBT.println(msg); 
}

// ==================== BT COMMANDS ====================
void handleBT(){
  while(SerialBT.available()){ char c=(char)SerialBT.read();
    if(c=='\n'||c=='\r'){ if(btBuffer.length()>0){processCommand(btBuffer);btBuffer="";} }
    else btBuffer+=c; }
  while(Serial.available()){ char c=(char)Serial.read();
    if(c=='\n'||c=='\r'){ if(btBuffer.length()>0){processCommand(btBuffer);btBuffer="";} }
    else btBuffer+=c; }
}
void processCommand(String cmd){
  cmd.trim(); cmd.toUpperCase(); if(cmd=="") return;
  if(cmd=="AT"){
    startRelayTune();
  } else if(cmd=="S"||cmd=="START"){
    isTuning=false; robotRunning=true; integral=0; lastError=0;
    sendBoth("Following: spd="+String(baseSpeed)+" Kp="+String(kp,2)+" Kd="+String(kd,2));
  } else if(cmd=="X"||cmd=="STOP"){
    isTuning=false; robotRunning=false; stopMotors(); sendBoth("STOPPED.");
  } else if(cmd=="FLIP"){
    invertSteering=!invertSteering;
    sendBoth("Steering: "+String(invertSteering?"INVERTED":"NORMAL"));
  } else if(cmd=="BEST"){
    sendBoth("=== BEST === Kp="+String(kp,2)+" Kd="+String(kd,2)+" Spd="+String(baseSpeed));
  } else if(cmd.startsWith("SPD ")){
    int v=cmd.substring(4).toInt();
    if(v>=MIN_SPEED&&v<=MAX_SPEED){ baseSpeed=v; sendBoth("Speed="+String(baseSpeed)); }
    else sendBoth("SPD range: "+String(MIN_SPEED)+"-"+String(MAX_SPEED));
  } else sendBoth("Unknown: "+cmd);
}
