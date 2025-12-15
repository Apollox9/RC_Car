#ifndef WIFI_READER_H
#define WIFI_READER_H

#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>

// ==========================================================
// CONFIGURATION - Edit these values for your network
// ==========================================================
#define WIFI_SSID "Airtel_X25A_72F5"
#define WIFI_PASSWORD "58BF8F59"
#define SERVER_IP "192.168.1.165"
#define SERVER_PORT 5050
#define DATA_ENDPOINT "/data"

// ==========================================================
// TIMING CONFIGURATION
// ==========================================================
#define WIFI_CONNECT_TIMEOUT_MS 10000 // Max time to wait for WiFi connection
#define WIFI_RECONNECT_INTERVAL_MS                                             \
  5000                         // Time between WiFi reconnection attempts
#define HTTP_TIMEOUT_MS 2000   // HTTP request timeout
#define COMMAND_TIMEOUT_MS 500 // If no command received, stop motors (safety)

// ==========================================================
// WiFiDrive Class - Handles all network communication
// ==========================================================
class WiFiDrive {
public:
  // Initialize WiFi connection with timeout
  static bool begin() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("[WiFi] Connecting to ");
    Serial.print(WIFI_SSID);

    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED) {
      if (millis() - startAttempt > WIFI_CONNECT_TIMEOUT_MS) {
        Serial.println("\n[WiFi] FAILED - Connection timeout!");
        return false;
      }
      delay(250);
      Serial.print(".");
    }

    Serial.println("\n[WiFi] Connected!");
    Serial.print("[WiFi] IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("[WiFi] Signal Strength (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");

    _lastFetch = 0;
    _lastCommandTime = millis();
    _latestData = "";
    _isConnected = true;

    return true;
  }

  // Check if WiFi is connected, attempt reconnection if not
  static bool checkConnection() {
    if (WiFi.status() == WL_CONNECTED) {
      _isConnected = true;
      return true;
    }

    _isConnected = false;
    _latestData = ""; // Clear data on disconnect (safety)

    // Attempt reconnection
    static unsigned long lastReconnectAttempt = 0;
    if (millis() - lastReconnectAttempt > WIFI_RECONNECT_INTERVAL_MS) {
      lastReconnectAttempt = millis();
      Serial.println("[WiFi] Disconnected! Attempting reconnection...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }

    return false;
  }

  // Call in loop - fetches data at specified interval
  static void update(unsigned long interval) {
    // Check WiFi connection first
    if (!checkConnection()) {
      return;
    }

    unsigned long now = millis();
    if (now - _lastFetch >= interval) {
      _lastFetch = now;
      fetchData();
    }
  }

  // Get latest received JSON data
  static String getLatestData() { return _latestData; }

  // Check if we have received a command recently (for safety timeout)
  static bool isCommandFresh() {
    return (millis() - _lastCommandTime) < COMMAND_TIMEOUT_MS;
  }

  // Check WiFi connection status
  static bool isConnected() { return _isConnected; }

  // Get signal strength
  static int getSignalStrength() { return WiFi.RSSI(); }

private:
  static void fetchData() {
    String url =
        String("http://") + SERVER_IP + ":" + SERVER_PORT + DATA_ENDPOINT;

    WiFiClient client;
    HTTPClient http;
    http.setTimeout(HTTP_TIMEOUT_MS);

    if (http.begin(client, url)) {
      int httpCode = http.GET();

      if (httpCode == HTTP_CODE_OK) {
        _latestData = http.getString();
        _lastCommandTime = millis(); // Update command timestamp
      } else if (httpCode > 0) {
        Serial.printf("[HTTP] Unexpected response code: %d\n", httpCode);
        _latestData = "";
      } else {
        Serial.printf("[HTTP] GET failed: %s\n",
                      http.errorToString(httpCode).c_str());
        _latestData = "";
      }
      http.end();
    } else {
      Serial.println("[HTTP] Unable to connect to server");
      _latestData = "";
    }
  }

  static unsigned long _lastFetch;
  static unsigned long _lastCommandTime;
  static String _latestData;
  static bool _isConnected;
};

// Static member initialization
unsigned long WiFiDrive::_lastFetch = 0;
unsigned long WiFiDrive::_lastCommandTime = 0;
String WiFiDrive::_latestData = "";
bool WiFiDrive::_isConnected = false;

// Function declarations
void WiFiLoop();
void WiFiSetup();

#endif
