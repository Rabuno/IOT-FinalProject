#include "Network.h"

#include <WiFi.h>

#include <ArduinoJson.h>
#include <PubSubClient.h>

#include <driver/gpio.h>
#include <freertos/task.h>

#include "../../include/rtos_globals.h"

namespace {

WiFiClient g_wifiClient;
PubSubClient g_mqtt(g_wifiClient);

Network::WifiMqttConfig g_cfg{};
uint32_t g_lastMqttAttemptMs = 0;

void setModeSafe(OperatingMode m) {
  if (g_semMode == nullptr) return;
  if (xSemaphoreTake(g_semMode, pdMS_TO_TICKS(20)) == pdTRUE) {
    g_currentMode = m;
    xSemaphoreGive(g_semMode);
  }
}

void setCmdSafe(DriveCmd c, uint32_t nowMs) {
  if (g_semCmd == nullptr) return;
  if (xSemaphoreTake(g_semCmd, pdMS_TO_TICKS(20)) == pdTRUE) {
    g_currentCmd = c;
    g_lastMqttCmdMs = nowMs;
    xSemaphoreGive(g_semCmd);
  }
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  (void)topic;
  if (length < 1) return;
  const char cmd = static_cast<char>(payload[0]);
  const uint32_t nowMs = millis();

  switch (cmd) {
    case 'M':  // MANUAL
      setModeSafe(OperatingMode::MANUAL);
      setCmdSafe(DriveCmd::STOP, nowMs);
      break;
    case 'A':  // AUTO
      setModeSafe(OperatingMode::AUTO);
      setCmdSafe(DriveCmd::STOP, nowMs);
      break;
    case 'F':
      setCmdSafe(DriveCmd::FORWARD, nowMs);
      break;
    case 'B':
      setCmdSafe(DriveCmd::BACKWARD, nowMs);
      break;
    case 'L':
      setCmdSafe(DriveCmd::LEFT, nowMs);
      break;
    case 'R':
      setCmdSafe(DriveCmd::RIGHT, nowMs);
      break;
    case 'S':
    default:
      setCmdSafe(DriveCmd::STOP, nowMs);
      break;
  }
}

void mqttEnsureConnected(uint32_t nowMs) {
  if (g_mqtt.connected()) return;
  if ((nowMs - g_lastMqttAttemptMs) < MQTT_RECONNECT_BACKOFF_MS) return;
  g_lastMqttAttemptMs = nowMs;

  if (g_cfg.mqttHost == nullptr || g_cfg.mqttHost[0] == '\0' || g_cfg.mqttClientId == nullptr) return;
  bool ok = false;
  if (g_cfg.mqttUsername != nullptr && g_cfg.mqttUsername[0] != '\0') {
    ok = g_mqtt.connect(g_cfg.mqttClientId, g_cfg.mqttUsername, (g_cfg.mqttPassword != nullptr) ? g_cfg.mqttPassword : "");
  } else {
    ok = g_mqtt.connect(g_cfg.mqttClientId);
  }
  if (!ok) return;

  g_mqtt.subscribe(MQTT_TOPIC_CONTROL, 0);

  // Adafruit IO feed subscription (optional)
  if (IO_USERNAME_STR[0] != '\0' && IO_KEY_STR[0] != '\0') {
    static char aioTopic[96];
    // "<username>/feeds/<feed>"
    snprintf(aioTopic, sizeof(aioTopic), "%s/feeds/%s", IO_USERNAME_STR, AIO_CONTROL_FEED);
    g_mqtt.subscribe(aioTopic, 0);
  }
}

// LoRa UART2
HardwareSerial& g_lora = Serial2;

volatile bool g_loraAuxHigh = true;
void IRAM_ATTR onLoraAuxIsr() {
  g_loraAuxHigh = (gpio_get_level(static_cast<gpio_num_t>(PIN_LORA_AUX)) == 1);
}

bool loraWaitAuxHigh(uint32_t timeoutMs) {
  const uint32_t start = millis();
  while (!g_loraAuxHigh) {
    if ((millis() - start) >= timeoutMs) return false;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return true;
}

}  // namespace

namespace Network {

void beginWifiMqtt(const WifiMqttConfig& cfg) {
  g_cfg = cfg;

  WiFi.mode(WIFI_STA);
  if (cfg.wifiSsid != nullptr && cfg.wifiSsid[0] != '\0') {
    WiFi.begin(cfg.wifiSsid, cfg.wifiPassword);
  }

  if (cfg.mqttHost != nullptr && cfg.mqttHost[0] != '\0') {
    g_mqtt.setServer(cfg.mqttHost, cfg.mqttPort);
  }
  g_mqtt.setCallback(onMqttMessage);
}

void mqttLoop(uint32_t nowMs) {
  if (WiFi.status() != WL_CONNECTED) {
    // Passive wait; keep task responsive.
    vTaskDelay(pdMS_TO_TICKS(50));
    return;
  }

  mqttEnsureConnected(nowMs);
  g_mqtt.loop();
}

void beginLoRa(uint32_t baud) {
  pinMode(PIN_LORA_AUX, INPUT);
  g_loraAuxHigh = (digitalRead(PIN_LORA_AUX) == HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_LORA_AUX), onLoraAuxIsr, CHANGE);

  g_lora.begin(baud, SERIAL_8N1, PIN_LORA_RXD_ESP, PIN_LORA_TXD_ESP);
}

const char* stateToString(VehicleState s) {
  switch (s) {
    case VehicleState::STATE_BOOT:
      return "BOOT";
    case VehicleState::STATE_IDLE:
      return "IDLE";
    case VehicleState::STATE_FORWARD:
      return "FORWARD";
    case VehicleState::STATE_AVOID:
      return "AVOID";
    case VehicleState::STATE_TRAPPED:
      return "TRAPPED";
    case VehicleState::STATE_FORCE_STOP:
      return "FORCE_STOP";
    case VehicleState::STATE_MANUAL:
      return "MANUAL";
    default:
      return "UNKNOWN";
  }
}

void loraSendAlert(const char* msg) {
  if (msg == nullptr) return;
  (void)loraWaitAuxHigh(50);
  g_lora.println(msg);
}

void loraSendTelemetry(float tempC, float humPct, uint8_t safetyIndex, VehicleState state) {
  StaticJsonDocument<192> doc;
  doc["Temp"] = tempC;
  doc["Hum"] = humPct;
  doc["SafetyIndex"] = safetyIndex;
  doc["State"] = stateToString(state);

  char out[192];
  const size_t n = serializeJson(doc, out, sizeof(out));
  if (n == 0) return;

  (void)loraWaitAuxHigh(50);
  g_lora.write(reinterpret_cast<const uint8_t*>(out), n);
  g_lora.write('\n');
}

}  // namespace Network
