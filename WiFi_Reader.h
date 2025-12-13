#ifndef WIFI_READER_H
#define WIFI_READER_H

#include <HTTPClient.h>
#include <WiFi.h>

class WiFiDrive {
public:
  static void begin(const char *ssid, const char *password,
                    const char *serverIP, uint16_t port) {
    _ssid = ssid;
    _password = password;
    _serverIP = serverIP;
    _port = port;

    WiFi.mode(WIFI_STA);
    WiFi.begin(_ssid, _password);

    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nWiFi connected.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    _lastFetch = 0;
    _latestData = "";
  }

  // Call in loop
  static void update(unsigned long interval) {
    unsigned long now = millis();
    if (now - _lastFetch >= interval) {
      _lastFetch = now;
      fetchData();
    }
  }

  static String getLatestData() { return _latestData; }

private:
  static void fetchData() {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi not connected!");
      return;
    }

    String url = String("http://") + _serverIP + ":" + _port + _path;
    WiFiClient client;
    HTTPClient http;

    // Use the modern begin(client, url) syntax
    if (http.begin(client, url)) {
      int httpCode = http.GET();

      if (httpCode > 0) {
        _latestData = http.getString();
      } else {
        Serial.print("HTTP GET failed, code: ");
        Serial.println(httpCode);
        Serial.println(http.errorToString(httpCode).c_str());
      }
      http.end();
    } else {
      Serial.println("Unable to connect (http.begin failed)");
    }
  }

  static const char *_ssid;
  static const char *_password;
  static const char *_serverIP;
  static uint16_t _port;
  static const char *_path;

  static unsigned long _lastFetch;
  static String _latestData;
};

// Static definitions
const char *WiFiDrive::_ssid = "Airtel_X25A_72F5";
const char *WiFiDrive::_password = "58BF8F59";
const char *WiFiDrive::_serverIP = "192.168.1.165";
uint16_t WiFiDrive::_port = 5050;
const char *WiFiDrive::_path = "/data";
unsigned long WiFiDrive::_lastFetch = 0;
String WiFiDrive::_latestData = "";
void WiFiLoop();
void WiFiSetup();

#endif
