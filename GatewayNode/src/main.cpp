#include <Arduino.h>
#include <WiFi.h>

#include <ArduinoJson.h>
#include <PubSubClient.h>

// =========================
// Build-time configuration
// =========================
// Set these via GatewayNode/platformio.ini build_flags:
//   -DWIFI_SSID=\"...\"
//   -DWIFI_PASS=\"...\"
//   -DIO_USERNAME=\"kichsatst\"
//   -DIO_KEY=\"...\"
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASS
#define WIFI_PASS ""
#endif
#ifndef IO_USERNAME
#define IO_USERNAME ""
#endif
#ifndef IO_KEY
#define IO_KEY ""
#endif

static constexpr const char* kWifiSsid = WIFI_SSID;
static constexpr const char* kWifiPass = WIFI_PASS;
static constexpr const char* kIoUser = IO_USERNAME;
static constexpr const char* kIoKey = IO_KEY;

static constexpr const char* kMqttHost = "io.adafruit.com";
static constexpr uint16_t kMqttPort = 1883;
static constexpr const char* kMqttClientId = "esp32-gateway";

// =========================
// LoRa UART (E32) wiring
// =========================
// Gateway ESP32 RX2 <- LoRa TXD, TX2 -> LoRa RXD
static constexpr int kLoraRxPin = 14;
static constexpr int kLoraTxPin = 4;
static constexpr uint32_t kLoraBaud = 9600;

// Robot telemetry JSON is newline-delimited.
static constexpr size_t kLineBufSize = 320;

WiFiClient g_wifi;
PubSubClient g_mqtt(g_wifi);

static void connectWifi() {
  if (kWifiSsid[0] == '\0') return;

  WiFi.mode(WIFI_STA);
  WiFi.begin(kWifiSsid, kWifiPass);

  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print('.');
    if ((millis() - start) > 30000) break;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK, IP=");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi NOT connected (timeout).");
  }
}

static bool mqttEnsureConnected() {
  if (g_mqtt.connected()) return true;
  if (kIoUser[0] == '\0' || kIoKey[0] == '\0') return false;
  Serial.print("MQTT connecting to ");
  Serial.print(kMqttHost);
  Serial.print(" as ");
  Serial.print(kIoUser);
  Serial.print(" ... ");
  const bool ok = g_mqtt.connect(kMqttClientId, kIoUser, kIoKey);
  Serial.println(ok ? "OK" : "FAIL");
  if (!ok) {
    Serial.print("MQTT state=");
    Serial.println(g_mqtt.state());
  }
  return ok;
}

static bool publishFeed(const char* feedKey, const char* value) {
  if (!mqttEnsureConnected()) return false;
  char topic[96];
  // "<username>/feeds/<feedKey>"
  snprintf(topic, sizeof(topic), "%s/feeds/%s", kIoUser, feedKey);
  return g_mqtt.publish(topic, value, true);
}

static bool publishFeedFloat(const char* feedKey, float value) {
  char buf[24];
  dtostrf(value, 0, 2, buf);
  return publishFeed(feedKey, buf);
}

static bool publishFeedU8(const char* feedKey, uint8_t value) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%u", static_cast<unsigned>(value));
  return publishFeed(feedKey, buf);
}

static void handleTelemetryLine(const char* line) {
  Serial.print("[LoRa] ");
  Serial.println(line);

  StaticJsonDocument<256> doc;
  const DeserializationError err = deserializeJson(doc, line);
  if (err) {
    Serial.print("[JSON] parse fail: ");
    Serial.println(err.c_str());
    return;
  }

  const float temp = doc["Temp"] | 0.0f;
  const float hum = doc["Hum"] | 0.0f;
  const uint8_t safety = doc["SafetyIndex"] | 0;
  const char* state = doc["State"] | "UNKNOWN";

  const bool ok1 = publishFeedFloat("temp", temp);
  const bool ok2 = publishFeedFloat("hum", hum);
  const bool ok3 = publishFeedU8("safetyindex", safety);
  const bool ok4 = publishFeed("state", state);
  Serial.print("[MQTT] publish: ");
  Serial.print(ok1 && ok2 && ok3 && ok4 ? "OK" : "FAIL");
  Serial.print(" (temp/hum/safetyindex/state)");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("ESP32 Gateway boot");

  Serial2.begin(kLoraBaud, SERIAL_8N1, kLoraRxPin, kLoraTxPin);
  Serial.println("LoRa UART2 ready (RX=14 TX=4 9600)");

  connectWifi();
  g_mqtt.setServer(kMqttHost, kMqttPort);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();
  }

  (void)mqttEnsureConnected();
  g_mqtt.loop();

  static char lineBuf[kLineBufSize];
  static size_t n = 0;

  while (Serial2.available()) {
    const int c = Serial2.read();
    if (c < 0) break;

    if (c == '\n') {
      lineBuf[n] = '\0';
      if (n > 0) handleTelemetryLine(lineBuf);
      n = 0;
      continue;
    }
    if (c == '\r') continue;

    if (n + 1 < sizeof(lineBuf)) {
      lineBuf[n++] = static_cast<char>(c);
    } else {
      // Drop oversized lines.
      n = 0;
    }
  }
}
