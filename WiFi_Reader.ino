#include "ArduinoJson.h"
#include "WiFi_Reader.h"
#include <Arduino.h>

// Fetch interval in milliseconds
const unsigned long FETCH_INTERVAL = 100;

// Forward declarations for motor functions defined in RC_Car.ino
extern void Park();
extern void Drive(String direction, float SpeedFactor, float SteeringFactor);

// ----------------------------
// Initialize WiFi connection
// ----------------------------
void WiFiSetup() {
  Serial.println("\n========================================");
  Serial.println("       ESP32 RC Car - WiFi Setup");
  Serial.println("========================================");

  if (!WiFiDrive::begin()) {
    Serial.println("[ERROR] WiFi setup failed! Motors will be disabled.");
    Serial.println("[ERROR] Check SSID/password in WiFi_Reader.h");
    // Don't block - allow car to still be used for testing
  }

  // Test server connectivity
  Serial.println("[Init] Testing server connection...");
  WiFiClient client;
  bool serverReachable = false;

  for (int i = 0; i < 3; i++) {
    if (client.connect(SERVER_IP, SERVER_PORT)) {
      serverReachable = true;
      client.stop();
      break;
    }
    Serial.printf("[Init] Server connection attempt %d/3 failed\n", i + 1);
    delay(500);
  }

  if (serverReachable) {
    Serial.println("[Init] Server is reachable!");
  } else {
    Serial.println(
        "[WARNING] Cannot reach server - check if server.py is running");
  }

  Serial.println("========================================");
  Serial.println("           Setup Complete!");
  Serial.println("========================================\n");
}

// ----------------------------
// Non-blocking WiFi loop
// ----------------------------
void WiFiLoop() {
  // Update WiFi data (handles reconnection internally)
  WiFiDrive::update(FETCH_INTERVAL);

  // Safety: If no fresh command, stop the car
  if (!WiFiDrive::isCommandFresh()) {
    static bool wasTimeout = false;
    if (!wasTimeout) {
      Serial.println("[SAFETY] Command timeout - stopping motors");
      Park();
      wasTimeout = true;
    }
    return; // Don't process stale data
  }

  // Reset timeout flag when we get fresh data
  static bool wasTimeout = false;
  wasTimeout = false;

  // Get latest JSON command
  String jsonCmd = WiFiDrive::getLatestData();
  if (jsonCmd.length() > 0) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, jsonCmd);
    if (!error) {
      String dir = doc["direction"] | "neutral"; // forward, reverse, neutral
      float speed = doc["speed"] | 0.0;          // 0.0 to 1.0
      float steer = doc["steering"] | 0.0;       // -1.0 to 1.0

      // Use your existing motor functions
      if (dir == "neutral") {
        Park();
      } else {
        Drive(dir, speed, steer);
      }
    } else {
      Serial.println("JSON parse error");
    }
  }
}


