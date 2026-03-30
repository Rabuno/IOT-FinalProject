#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <Wire.h>

#include "HAL.h"
#include "Network.h"
#include "config.h"
#include "rtos_globals.h"

// Task entrypoints (implemented in src/tasks.cpp)
void taskMQTT(void* pv);
void taskLoRa(void* pv);
void taskSensors(void* pv);
void taskMotorFSM(void* pv);

namespace {

void createIpc() {
  g_semMode = xSemaphoreCreateMutex();
  g_semCmd = xSemaphoreCreateMutex();
  g_semSafety = xSemaphoreCreateMutex();
  g_semScan = xSemaphoreCreateMutex();
  g_semEnv = xSemaphoreCreateMutex();
  g_semState = xSemaphoreCreateMutex();

  // Priority alerts for LoRa
  g_qLoRaPriority = xQueueCreate(8, sizeof(char[96]));

  // Seed time for deadman
  if (g_semCmd && xSemaphoreTake(g_semCmd, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_lastMqttCmdMs = millis();
    xSemaphoreGive(g_semCmd);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);

  createIpc();

  HAL::initPins();
  HAL::initPwm();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // Network init (credentials provided via build flags / config.h macros)
  Network::WifiMqttConfig cfg{};
  cfg.wifiSsid = WIFI_SSID_STR;
  cfg.wifiPassword = WIFI_PASS_STR;
  // If Adafruit IO credentials are provided, default to Adafruit IO broker.
  cfg.mqttHost = (IO_USERNAME_STR[0] != '\0' && IO_KEY_STR[0] != '\0') ? AIO_MQTT_HOST : MQTT_HOST_STR;
  cfg.mqttPort = MQTT_PORT;
  cfg.mqttClientId = MQTT_CLIENT_ID;
  const bool hasAioCreds = (IO_USERNAME_STR[0] != '\0' && IO_KEY_STR[0] != '\0');
  cfg.mqttUsername = hasAioCreds ? IO_USERNAME_STR : nullptr;
  cfg.mqttPassword = hasAioCreds ? IO_KEY_STR : nullptr;

  Network::beginWifiMqtt(cfg);
  Network::beginLoRa(9600);

  // Core 0: Networking
  xTaskCreatePinnedToCore(taskMQTT, "taskMQTT", 4096, nullptr, 2, nullptr, CORE_NET);
  xTaskCreatePinnedToCore(taskLoRa, "taskLoRa", 4096, nullptr, 1, nullptr, CORE_NET);

  // Core 1: Sensors + Control
  xTaskCreatePinnedToCore(taskSensors, "taskSensors", 4096, nullptr, 2, nullptr, CORE_CTRL);
  xTaskCreatePinnedToCore(taskMotorFSM, "taskMotorFSM", 4096, nullptr, 3, nullptr, CORE_CTRL);
}

void loop() {
  // All work is done in FreeRTOS tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
