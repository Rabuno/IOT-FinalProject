#pragma once

#include <Arduino.h>

#include "../../include/config.h"

namespace Network {

struct WifiMqttConfig {
  const char* wifiSsid;
  const char* wifiPassword;
  const char* mqttHost;
  uint16_t mqttPort;
  const char* mqttClientId;
  const char* mqttUsername;  // optional (e.g., Adafruit IO)
  const char* mqttPassword;  // optional (e.g., Adafruit IO key)
};

// Call once from setup().
void beginWifiMqtt(const WifiMqttConfig& cfg);
void beginLoRa(uint32_t baud = 9600);

// Call from taskMQTT loop.
void mqttLoop(uint32_t nowMs);

// LoRa send helpers (call from taskLoRa).
void loraSendTelemetry(float tempC, float humPct, uint8_t safetyIndex, VehicleState state);
void loraSendAlert(const char* msg);

const char* stateToString(VehicleState s);

}  // namespace Network
