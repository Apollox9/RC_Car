#include "WiFi_Reader.h"
#include <Arduino.h>
// ==========================================================
#define PWM_FREQ 5000
#define PWM_RES 8 // 8-bit resolution
#define CH_LEFT 0
#define CH_RIGHT 1
// MOTOR PINS
// ==========================================================
#define IN1_L 14
#define IN2_L 27
#define EN_L 26 // PWM Left

#define IN1_R 25
#define IN2_R 33
#define EN_R 32 // PWM Right

// ==========================================================
// ULTRASONIC SENSORS
// ==========================================================
#define TRIG_FRONT 4
#define ECHO_FRONT 2
#define TRIG_BACK 18
#define ECHO_BACK 19

bool Obstacle_detection_mode = false;
float stopDistance = 20.0; // cm safe distance

// ==========================================================
// DRIVING STATE
// ==========================================================
enum DriveMode { MODE_NEUTRAL, MODE_DRIVE };
DriveMode currentMode = MODE_NEUTRAL;

float currentLeftPWM = 0;
float currentRightPWM = 0;
float rampSpeed = 0.03;
unsigned long lastRampTime = 0;

// Optional PID steering
bool enablePIDSteering = false;
float Kp = 0.6, Ki = 0.01, Kd = 0.1;
float pidIntegral = 0;
float pidLastErr = 0;

// ==========================================================
// PWM (Simple universal analogWrite)
// ==========================================================

void setPWM(int channel, float value) {
  int pwmVal = constrain((int)(value * 255), 0, 255);
  ledcWrite(channel, pwmVal);
}

void writePWM(int pin, int value) { analogWrite(pin, value); }

void setupPWM() {
  ledcSetup(CH_LEFT, PWM_FREQ, PWM_RES);
  ledcAttachPin(EN_L, CH_LEFT);

  ledcSetup(CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(EN_R, CH_RIGHT);
}

// ==========================================================
// SETUP
// ==========================================================
void setup() {
  Serial.begin(115200);

  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);

  setupPWM();
  setupPWM();

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_BACK, OUTPUT);
  pinMode(ECHO_BACK, INPUT);

  Serial.println("System Ready.");
  // InputSetup();
  WiFiSetup();
  setupPWM();
}

// ==========================================================
// ULTRASONIC SENSOR
// ==========================================================
float readUltrasonic(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 25000);
  if (duration == 0)
    return 999;

  return duration * 0.0343 / 2.0;
}

// ==========================================================
// MOTOR CONTROL
// ==========================================================
void setMotorRaw(bool left, int direction, float pwm) {
  int IN1 = left ? IN1_L : IN1_R;
  int IN2 = left ? IN2_L : IN2_R;
  int PWM_PIN = left ? EN_L : EN_R;

  if (direction == 1) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (direction == -1) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  int pwmVal = constrain(pwm * 255, 0, 255);
  writePWM(PWM_PIN, pwmVal);
}

// ==========================================================
// PARK MODE
// ==========================================================
void Park() {
  currentMode = MODE_NEUTRAL;

  setMotorRaw(true, 0, 0);
  setMotorRaw(false, 0, 0);

  currentLeftPWM = 0;
  currentRightPWM = 0;

  Serial.println("MODE: NEUTRAL (motors stopped)");
}

// ==========================================================
// RAMPING
// ==========================================================
float rampTo(float current, float target) {
  if (millis() - lastRampTime < 10)
    return current;
  lastRampTime = millis();

  if (current < target)
    current += rampSpeed;
  else if (current > target)
    current -= rampSpeed;

  return constrain(current, 0, 1);
}

// ==========================================================
// DRIVE FUNCTION
// ==========================================================
void Drive(String direction, float SpeedFactor, float SteeringFactor) {

  if (currentMode == MODE_NEUTRAL)
    return;

  // Safety
  if (Obstacle_detection_mode) {
    float front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
    float back = readUltrasonic(TRIG_BACK, ECHO_BACK);

    if (direction == "forward" && front < stopDistance) {
      Serial.println("Front Blocked!");
      Park();
      return;
    }

    if (direction == "reverse" && back < stopDistance) {
      Serial.println("Rear Blocked!");
      Park();
      return;
    }
  }

  // PID (optional)
  if (enablePIDSteering) {
    float error = SteeringFactor;
    pidIntegral += error;
    float derivative = error - pidLastErr;
    pidLastErr = error;

    SteeringFactor =
        constrain(Kp * error + Ki * pidIntegral + Kd * derivative, -1, 1);
  }

  float leftTarget = SpeedFactor * (1 - SteeringFactor);
  float rightTarget = SpeedFactor * (1 + SteeringFactor);

  leftTarget = constrain(leftTarget, 0.0, 1.0);
  rightTarget = constrain(rightTarget, 0.0, 1.0);

  currentLeftPWM = rampTo(currentLeftPWM, leftTarget);
  currentRightPWM = rampTo(currentRightPWM, rightTarget);

  int dir = (direction == "forward") ? 1 : -1;

  setMotorRaw(true, dir, currentLeftPWM);
  setMotorRaw(false, dir, currentRightPWM);

  Serial.printf("Drive %s  L=%.2f  R=%.2f\n", direction.c_str(), currentLeftPWM,
                currentRightPWM);
}

// Direct motor drive function
// leftSpeed and rightSpeed range from -1.0 (full reverse) to 1.0 (full forward)
void DriveMotorsDirect(float leftSpeed, float rightSpeed) {
  // Left
  if (leftSpeed > 0) {
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
  } else {
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, LOW);
  }
  setPWM(CH_LEFT, abs(leftSpeed));

  // Right
  if (rightSpeed > 0) {
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
  } else {
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);
  }
  setPWM(CH_RIGHT, abs(rightSpeed));
}

// ==========================================================
// MAIN LOOP
// ==========================================================
unsigned long lastUpdate = 0;
const unsigned long INTERVAL = 50; // 50 ms
int step = 0;

void loop() {
  unsigned long now = millis();

  // Update sensors, WiFi, PID every loop
  WiFiLoop();
  // InputLoop();

  // Run motor sequence every INTERVAL
  if (now - lastUpdate >= INTERVAL) {
    lastUpdate = now;

    //   switch(step) {
    //     case 0: Drive("forward", 0.5, 0.0); break;
    //     case 1: Drive("forward", 0.6, -0.5); break;
    //     case 2: Drive("forward", 0.6, 0.5); break;
    //     case 3: Park(); break;
    //   }

    //   step = (step + 1) % 4;
    // }

    // Loop runs as fast as possible
  }
