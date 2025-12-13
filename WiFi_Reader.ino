#include "ArduinoJson.h"
#include "WiFi_Reader.h"
#include <Arduino.h>

// Fetch interval in milliseconds
const unsigned long FETCH_INTERVAL = 100;

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

  // Tiny delay to yield
  delay(1);
}
