#pragma once

#include <Arduino.h>

// =========================
// Pin Mapping (DO NOT EDIT)
// =========================
// Motor driver (L298N)
#define PIN_MOTOR_ENA 19
#define PIN_MOTOR_IN1 18
#define PIN_MOTOR_IN2 27
#define PIN_MOTOR_IN3 26
#define PIN_MOTOR_IN4 25
#define PIN_MOTOR_ENB 33

// Servo (Radar)
#define PIN_SERVO 13

// Ultrasonic (SRF05) - Echo is input-only (GPIO35) and has an external voltage divider
#define PIN_US_TRIG 32
#define PIN_US_ECHO 35

// LoRa E32 (UART2)
#define PIN_LORA_RXD_ESP 14 // ESP32 RXD2 <- LoRa TXD
#define PIN_LORA_TXD_ESP 4  // ESP32 TXD2 -> LoRa RXD
#define PIN_LORA_AUX 34     // Input-only, hardware interrupt

// DHT11
#define PIN_DHT 23

// LCD I2C 1602 (Hardware I2C)
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

// =========================
// LEDC / PWM Configuration
// =========================
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 1000;
constexpr uint8_t MOTOR_PWM_RES_BITS = 8; // 0..255 duty
constexpr uint8_t MOTOR_LEDC_CH_ENA = 0;  // ENA -> Channel 0
constexpr uint8_t MOTOR_LEDC_CH_ENB = 1;  // ENB -> Channel 1

constexpr uint32_t SERVO_PWM_FREQ_HZ = 50;
constexpr uint8_t SERVO_PWM_RES_BITS = 16; // finer pulse control
constexpr uint8_t SERVO_LEDC_CH = 2;       // Servo -> Channel 2

// Common servo pulse range (microseconds). Adjust mechanically if needed.
constexpr uint16_t SERVO_PULSE_MIN_US = 500;
constexpr uint16_t SERVO_PULSE_MAX_US = 2500;

// Servo mechanical center calibration (degrees).
// If the sensor is physically offset (e.g., points 3–5° off-center at "90°"),
// set this to a signed correction (negative/positive) to re-center.
constexpr int8_t SERVO_CENTER_OFFSET_DEG = 20;

constexpr uint8_t clampServoDeg(int deg)
{
  return static_cast<uint8_t>((deg < 0) ? 0 : (deg > 180) ? 180
                                                          : deg);
}

constexpr uint8_t SERVO_CENTER_DEG = clampServoDeg(90 + SERVO_CENTER_OFFSET_DEG);

// =========================
// Sensors / Timing Constants
// =========================
constexpr uint8_t RADAR_STEPS = 7;
constexpr uint8_t RADAR_ANGLES_DEG[RADAR_STEPS] = {0, 30, 60, 90, 120, 150, 180};
// Servo settle time before ping (must be 120–200ms as per requirement).
constexpr uint32_t RADAR_SETTLE_MS = 180;
constexpr uint32_t US_ECHO_TIMEOUT_US = 30000; // ~5m max; keeps ping bounded
// Front distance refresh while cruising forward (servo held at center, single ping).
constexpr uint32_t FRONT_PING_PERIOD_MS = 80;

// DHT11
constexpr uint32_t DHT_SAMPLE_PERIOD_MS = 2000;

// LCD 1602
constexpr uint8_t LCD_I2C_ADDR = 0x27;
constexpr uint8_t LCD_COLS = 16;
constexpr uint8_t LCD_ROWS = 2;

// =========================
// Safety Index (S) Settings
// =========================
// Safety index calculation:
//   S = floor(0.6*sD + 0.3*sT + 0.1*sH)
constexpr uint8_t SAFETY_FORCE_STOP_THRESHOLD = 5; // Veto rule: S <= 3 => FORCE_STOP

// Distance buckets (cm)
constexpr uint16_t DIST_BUCKET_NEAR_CM = 15;
constexpr uint16_t DIST_BUCKET_MID_CM = 50;

// Temperature buckets (°C)
constexpr float TEMP_BUCKET_HOT_C = 50.0f;
constexpr float TEMP_BUCKET_WARM_C = 35.0f;

// Humidity thresholds (%RH)
constexpr float HUM_BUCKET_HIGH_PCT = 85.0f;
constexpr float HUM_BUCKET_LOW_PCT = 20.0f;

// Obstacle hysteresis (cm)
constexpr uint16_t AVOID_TRIGGER_CM = 40;
constexpr uint16_t FORWARD_RESUME_CM = 55;

// Avoidance watchdog: if stuck in AVOID too long without clearing, force TRAPPED escape.
constexpr uint32_t AVOID_TIMEOUT_MS = 2500;

// Forward situational awareness scan (reduces "tunnel vision" without running radar continuously).
// Runs a full 7-point scan periodically while cruising forward in AUTO.
constexpr uint32_t FORWARD_SCAN_PERIOD_MS = 2500;

// =========================
// Anti-Trap / Escape Logic
// =========================
constexpr uint8_t ANTI_TRAP_TRANSITION_LIMIT = 3;
constexpr uint32_t ANTI_TRAP_WINDOW_MS = 5000;

constexpr uint8_t TRAPPED_TWICE_LIMIT = 2;
constexpr uint32_t TRAPPED_TWICE_WINDOW_MS = 15000;

constexpr uint32_t ESCAPE_REVERSE_MS = 1500;
constexpr uint32_t ESCAPE_HARD_TURN_MS = 800;
// Escape PWM tuning:
// - Reverse is kept conservative by default (no rear sensor).
// - Allow faster reverse only when deeply trapped (very small front clearance).
constexpr uint8_t ESCAPE_TURN_PWM_PCT = 90;
constexpr uint8_t ESCAPE_REVERSE_PWM_SLOW_PCT = 35;
constexpr uint8_t ESCAPE_REVERSE_PWM_FAST_PCT = 55;
constexpr uint16_t ESCAPE_REVERSE_FAST_IF_FRONT_LT_CM = 10;

// =========================
// Motion Speed Profiles
// =========================
// These are the default PWM percentages used by the motor task.
// Tune down if the chassis is too fast / unstable.
constexpr uint8_t PWM_AUTO_FWD_PCT = 70;
constexpr uint8_t PWM_AUTO_TURN_PCT = 90;
constexpr uint8_t PWM_MANUAL_FWD_PCT = 55;
constexpr uint8_t PWM_MANUAL_TURN_PCT = 60;
// Reverse is ALWAYS slower (no rear sensor).
constexpr uint8_t PWM_REVERSE_PCT = 60;

// Kinematic stall detection
constexpr uint32_t KIN_STALL_WINDOW_MS = 2000;
constexpr uint16_t KIN_STALL_MIN_DELTA_CM = 2;
// Only evaluate stall when ultrasonic distance is valid and within this range.
// Prevents false stalls when SRF05 returns 0 (timeout) or saturates at far distances.
constexpr uint16_t KIN_STALL_MAX_DISTANCE_CM = 200;
// Arm stall detection only when an obstacle is "in sight" (otherwise distance may not change in open space).
constexpr uint16_t KIN_STALL_ARM_DISTANCE_CM = 120;

// Manual deadman switch
constexpr uint32_t MANUAL_DEADMAN_MS = 1500;
// Auto-return from MANUAL when no control commands have been received for this long.
// Prevents getting stuck in MANUAL after an automatic fallback (e.g., trapped twice).
constexpr uint32_t MANUAL_AUTO_RETURN_MS = 8000;

// =========================
// Networking / Telemetry
// =========================
// MQTT control
constexpr uint16_t MQTT_PORT = 1883;
constexpr const char *MQTT_TOPIC_CONTROL = "bot/control";
constexpr uint32_t MQTT_RECONNECT_BACKOFF_MS = 1000;

// Configure these in your build flags or by editing here.
// Example build_flags:
//   -DWIFI_SSID=\"yourssid\" -DWIFI_PASS=\"yourpass\" -DMQTT_HOST=\"192.168.1.10\"
#ifndef WIFI_SSID
#define WIFI_SSID "Rabuno"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "00000000"
#endif
#ifndef MQTT_HOST
#define MQTT_HOST ""
#endif

constexpr const char *WIFI_SSID_STR = WIFI_SSID;
constexpr const char *WIFI_PASS_STR = WIFI_PASS;
constexpr const char *MQTT_HOST_STR = MQTT_HOST;
constexpr const char *MQTT_CLIENT_ID = "esp32-vehicle";

// Adafruit IO MQTT (optional)
// Set these via build flags to avoid committing secrets:
//   -DIO_USERNAME=\"kichsatst\" -DIO_KEY=\"<your_adafruit_io_key>\"
#ifndef IO_USERNAME
#define IO_USERNAME "kichsatst"
#endif
#ifndef IO_KEY
#define IO_KEY "aio_YDlO69ma4vRC3cVkDqWvXoKt789s"
#endif

constexpr const char *IO_USERNAME_STR = IO_USERNAME;
constexpr const char *IO_KEY_STR = IO_KEY;
constexpr const char *AIO_MQTT_HOST = "io.adafruit.com";
// Feed key for control commands (Adafruit IO feed keys must not contain '/')
constexpr const char *AIO_CONTROL_FEED = "bot-control";

// LoRa telemetry interval
constexpr uint32_t LORA_TELEMETRY_PERIOD_MS = 5000;

// =========================
// FreeRTOS Core Assignment
// =========================
constexpr int CORE_NET = 0;  // taskMQTT, taskLoRa
constexpr int CORE_CTRL = 1; // taskSensors, taskMotorFSM

// =========================
// Enums (System Contracts)
// =========================
enum class OperatingMode : uint8_t
{
  AUTO = 0,
  MANUAL = 1,
};

enum class DriveCmd : uint8_t
{
  STOP = 0,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
};

enum class VehicleState : uint8_t
{
  STATE_BOOT = 0,
  STATE_IDLE,
  STATE_FORWARD,
  STATE_AVOID,
  STATE_TRAPPED,
  STATE_FORCE_STOP,
  STATE_MANUAL,
};

// Convenience helpers
constexpr uint8_t pwmPctToDuty(uint8_t pct)
{
  return static_cast<uint8_t>((static_cast<uint16_t>(pct) * 255u) / 100u);
}
