// ==========================================================
//                  ESP32 RC CAR CONTROLLER
//                   Industry-Grade Version
// ==========================================================
// Features:
// - Dual motor control with ramping
// - WiFi command interface
// - Watchdog timer for safety
// - Ultrasonic obstacle detection (optional)
// - PID steering (optional)
// ==========================================================

#include "WiFi_Reader.h"
#include <Arduino.h>
#include <esp_task_wdt.h> // Watchdog timer

// ==========================================================
// HARDWARE CONFIGURATION
// ==========================================================

// PWM Configuration
#define PWM_FREQ 5000 // 5kHz PWM frequency
#define PWM_RES 8     // 8-bit resolution (0-255)
#define CH_LEFT 0     // LEDC channel for left motor
#define CH_RIGHT 1    // LEDC channel for right motor

// Motor Driver Pins (L298N or similar)
#define IN1_L 14 // Left motor direction pin 1
#define IN2_L 27 // Left motor direction pin 2
#define EN_L 26  // Left motor PWM enable

#define IN1_R 25 // Right motor direction pin 1
#define IN2_R 33 // Right motor direction pin 2
#define EN_R 32  // Right motor PWM enable

// Ultrasonic Sensor Pins (optional)
#define TRIG_FRONT 4
#define ECHO_FRONT 2
#define TRIG_BACK 18
#define ECHO_BACK 19

// ==========================================================
// SAFETY CONFIGURATION
// ==========================================================
#define WATCHDOG_TIMEOUT_S 5           // Watchdog timeout in seconds
#define MIN_PWM_THRESHOLD 0.05f        // Minimum PWM to actually drive motors
bool obstacleDetectionEnabled = false; // Enable ultrasonic safety
float stopDistance = 20.0f;            // Obstacle distance threshold (cm)

// ==========================================================
// DRIVING STATE
// ==========================================================
enum DriveMode { MODE_NEUTRAL, MODE_DRIVE };
volatile DriveMode currentMode = MODE_NEUTRAL;

float currentLeftPWM = 0.0f;
float currentRightPWM = 0.0f;
float rampSpeed = 0.15f; // PWM change per update cycle

// ==========================================================
// PID STEERING (Optional)
// ==========================================================
bool enablePIDSteering = false;
float Kp = 0.6f, Ki = 0.01f, Kd = 0.1f;
float pidIntegral = 0.0f;
float pidLastErr = 0.0f;
#define PID_INTEGRAL_LIMIT 10.0f // Prevent integral windup

// ==========================================================
// PWM SETUP
// ==========================================================
void setupPWM() {
  // Configure LEDC channels for motor PWM
  ledcSetup(CH_LEFT, PWM_FREQ, PWM_RES);
  ledcAttachPin(EN_L, CH_LEFT);

  ledcSetup(CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(EN_R, CH_RIGHT);

  // Start with motors stopped
  ledcWrite(CH_LEFT, 0);
  ledcWrite(CH_RIGHT, 0);

  Serial.println("[PWM] Motor PWM channels initialized");
}

// ==========================================================
// MOTOR CONTROL - Low Level
// ==========================================================
void setMotorRaw(bool isLeft, int direction, float pwm) {
  // Select pins based on motor side
  const int IN1 = isLeft ? IN1_L : IN1_R;
  const int IN2 = isLeft ? IN2_L : IN2_R;
  const int channel = isLeft ? CH_LEFT : CH_RIGHT;

  // Set motor direction
  switch (direction) {
  case 1: // Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    break;
  case -1: // Reverse
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    break;
  default: // Stop/Brake
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    break;
  }

  // Apply PWM (ensure valid range)
  int pwmVal = constrain((int)(pwm * 255.0f), 0, 255);
  ledcWrite(channel, pwmVal);
}

// Stop both motors immediately
void stopMotors() {
  setMotorRaw(true, 0, 0);  // Left motor
  setMotorRaw(false, 0, 0); // Right motor
  currentLeftPWM = 0.0f;
  currentRightPWM = 0.0f;
}

// ==========================================================
// PARK MODE - Safe stop
// ==========================================================
void Park() {
  if (currentMode != MODE_NEUTRAL) {
    Serial.println("[Motor] Entering PARK mode");
  }

  currentMode = MODE_NEUTRAL;
  stopMotors();

  // Reset PID state
  pidIntegral = 0.0f;
  pidLastErr = 0.0f;
}

// ==========================================================
// RAMPING - Smooth acceleration/deceleration
// ==========================================================
inline float rampTo(float current, float target) {
  if (current < target) {
    return min(current + rampSpeed, target);
  } else if (current > target) {
    return max(current - rampSpeed, target);
  }
  return current;
}

// ==========================================================
// ULTRASONIC SENSOR
// ==========================================================
float readUltrasonic(int trigPin, int echoPin) {
  // Trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure echo time (with timeout)
  long duration = pulseIn(echoPin, HIGH, 25000); // 25ms timeout

  if (duration == 0) {
    return 999.0f; // No echo received
  }

  // Convert to distance in cm
  return (duration * 0.0343f) / 2.0f;
}

// ==========================================================
// DRIVE FUNCTION - Main motor control
// ==========================================================
void Drive(String direction, float speedFactor, float steeringFactor) {
  // Validate direction and activate drive mode
  if (direction != "forward" && direction != "reverse") {
    Park();
    return;
  }

  currentMode = MODE_DRIVE;

  // Reset watchdog - we're actively driving
  esp_task_wdt_reset();

  // Obstacle detection (if enabled)
  if (obstacleDetectionEnabled) {
    float frontDist = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
    float backDist = readUltrasonic(TRIG_BACK, ECHO_BACK);

    if (direction == "forward" && frontDist < stopDistance) {
      Serial.printf("[Safety] Obstacle ahead at %.1f cm!\n", frontDist);
      Park();
      return;
    }

    if (direction == "reverse" && backDist < stopDistance) {
      Serial.printf("[Safety] Obstacle behind at %.1f cm!\n", backDist);
      Park();
      return;
    }
  }

  // Apply PID steering correction (if enabled)
  if (enablePIDSteering) {
    float error = steeringFactor;
    pidIntegral =
        constrain(pidIntegral + error, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    float derivative = error - pidLastErr;
    pidLastErr = error;

    steeringFactor =
        constrain(Kp * error + Ki * pidIntegral + Kd * derivative, -1.0f, 1.0f);
  }

  // Calculate target speeds for differential steering
  float leftTarget = speedFactor * (1.0f - steeringFactor);
  float rightTarget = speedFactor * (1.0f + steeringFactor);

  // Clamp to valid range
  leftTarget = constrain(leftTarget, 0.0f, 1.0f);
  rightTarget = constrain(rightTarget, 0.0f, 1.0f);

  // Apply ramping for smooth acceleration
  currentLeftPWM = rampTo(currentLeftPWM, leftTarget);
  currentRightPWM = rampTo(currentRightPWM, rightTarget);

  // Determine motor direction
  int dir = (direction == "forward") ? 1 : -1;

  // Apply minimum threshold - prevent motor stall
  float leftPWM = (currentLeftPWM < MIN_PWM_THRESHOLD) ? 0.0f : currentLeftPWM;
  float rightPWM =
      (currentRightPWM < MIN_PWM_THRESHOLD) ? 0.0f : currentRightPWM;

  // Drive motors
  setMotorRaw(true, dir, leftPWM);
  setMotorRaw(false, dir, rightPWM);

  // Debug output (only when actually moving)
  static unsigned long lastDebugPrint = 0;
  if (millis() - lastDebugPrint > 500) { // Print every 500ms max
    Serial.printf("[Drive] %s L=%.2f R=%.2f\n", direction.c_str(), leftPWM,
                  rightPWM);
    lastDebugPrint = millis();
  }
}

// ==========================================================
// SETUP
// ==========================================================
void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(100); // Allow serial to initialize

  Serial.println("\n");
  Serial.println("==========================================");
  Serial.println("    ESP32 RC Car Controller v2.0");
  Serial.println("==========================================");

  // Configure motor driver pins
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);

  // Initialize PWM
  setupPWM();

  // Ensure motors are stopped at startup
  stopMotors();

  // Configure ultrasonic sensor pins (optional)
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_BACK, OUTPUT);
  pinMode(ECHO_BACK, INPUT);

  // Initialize watchdog timer for safety
  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true); // Enable panic on timeout
  esp_task_wdt_add(NULL);                      // Add current task to watchdog
  Serial.printf("[Watchdog] Initialized with %d second timeout\n",
                WATCHDOG_TIMEOUT_S);

  // Initialize WiFi connection
  WiFiSetup();

  Serial.println("[System] Initialization complete - Ready!");
  Serial.println("==========================================\n");
}

// ==========================================================
// MAIN LOOP
// ==========================================================
void loop() {
  // Reset watchdog timer
  esp_task_wdt_reset();

  // Process WiFi commands (handles motor control internally)
  WiFiLoop();

  // Small yield to prevent watchdog issues
  delay(1);
}
