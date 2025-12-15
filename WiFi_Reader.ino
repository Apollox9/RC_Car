#include "ArduinoJson.h"
#include "WiFi_Reader.h"
#include <Arduino.h>

// Fetch interval in milliseconds
const unsigned long FETCH_INTERVAL = 100;
//Helper
float clampf(float v, float minVal, float maxVal) {
  if (v < minVal) return minVal;
  if (v > maxVal) return maxVal;
  return v;
}
//Drive
void DriveMotorsArcade(String direction, float speed, float steer) {
  // Clamp inputs
  speed = clampf(speed, 0.0f, 1.0f);
  steer = clampf(steer, -1.0f, 1.0f);

  // Convert direction + speed to signed throttle
  float throttle = 0.0f;

  if (direction == "forward") {
    throttle = speed;
  } 
  else if (direction == "reverse") {
    throttle = -speed;
  } 
  else { // "neutral"
    throttle = 0.0f;
  }

  // Differential mixing
  float left  = throttle + steer;
  float right = throttle - steer;

  // Normalize so we never exceed [-1, 1]
  float maxMag = max(abs(left), abs(right));
  if (maxMag > 1.0f) {
    left  /= maxMag;
    right /= maxMag;
  }

  // Drive motors
  Serial.print("left value: ");
  Serial.print(left);  
  Serial.print(" right value: ");
  Serial.println(right);
  DriveMotorsDirect(left, right);
}
// ----------------------------
// Initialize WiFi connection
// Call once in main setup()
// ----------------------------
void WiFiSetup() {
  Serial.println("Connecting to WiFi...");

  // Connect to WiFi
  WiFiDrive::begin("Airtel_X25A_72F5", "58BF8F59", "192.168.1.165", 5050);

  // Print ESP32 IP
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // Test connection to server
  Serial.println("Pinging server...");
  WiFiClient client;
  bool connected = false;

  for (int i = 0; i < 5; i++) {
    if (client.connect("192.168.1.165", 5050)) {
      connected = true;
      break;
    }
    Serial.println("Retry connection...");
    delay(500);
  }

  if (!connected) {
    Serial.println("Cannot connect to server!");
  } else {
    Serial.println("Server reachable!");
    client.stop(); // close connection
  }
}

// ----------------------------
// Non-blocking fetch from PC
// Call repeatedly in main loop()
// ----------------------------
void WiFiLoop() {
  // Update WiFi data
  WiFiDrive::update(FETCH_INTERVAL);

  // Get latest JSON command
  String jsonCmd = WiFiDrive::getLatestData();
  if (jsonCmd.length() > 0) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, jsonCmd);
      // if (error) {
      //   Serial.print("deserializeJson() failed: ");
      //   Serial.println(error.c_str());
      //   return;
      // }

      // serializeJson(doc, Serial);
      // Serial.println();  // newline    
      if (!error) {
        // String dir = doc["direction"] | "neutral"; // forward, reverse, neutral
        // float speed = doc["speed"] | 0.0;          // 0.0 to 1.0
        // float steer = doc["steering"] | 0.0;       // -1.0 to 1.0
        String dir = doc["direction"].as<String>(); // forward, reverse, neutral
        float speed = doc["speed"].as<float>();          // 0.0 to 1.0
        float steer = doc["steering"].as<float>();       // -1.0 to 1.0        

        // Use your existing motor functions
        if (dir == "neutral") {
          Park();
        } else {
          //Drive(dir, speed, steer);
          DriveMotorsArcade(dir, speed, steer);
        }
      } else {
        Serial.println("JSON parse error");
      }
  }

  // Tiny delay to yield
  delay(1);
}


