#include <Arduino.h>

#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "HAL.h"
#include "Network.h"
#include "config.h"
#include "rtos_globals.h"

// =========================
// Globals (Definitions)
// =========================
SemaphoreHandle_t g_semMode = nullptr;
SemaphoreHandle_t g_semCmd = nullptr;
SemaphoreHandle_t g_semSafety = nullptr;
SemaphoreHandle_t g_semScan = nullptr;
SemaphoreHandle_t g_semEnv = nullptr;
SemaphoreHandle_t g_semState = nullptr;

OperatingMode g_currentMode = OperatingMode::AUTO;
DriveCmd g_currentCmd = DriveCmd::STOP;
uint8_t g_safetyIndex = 9;
uint16_t g_scanArray[RADAR_STEPS] = {0};
float g_tempC = NAN;
float g_humPct = NAN;
VehicleState g_vehicleState = VehicleState::STATE_BOOT;

uint32_t g_lastMqttCmdMs = 0;

QueueHandle_t g_qLoRaPriority = nullptr;

namespace {

constexpr uint32_t SENSORS_TICK_MS = 50;
constexpr uint32_t RADAR_TRIGGER_COOLDOWN_MS = 900;
constexpr uint32_t LCD_REFRESH_MS = 500;

bool queueAlert(const char* msg) {
  if (g_qLoRaPriority == nullptr || msg == nullptr) return false;
  char buf[96] = {0};
  strlcpy(buf, msg, sizeof(buf));
  return xQueueSend(g_qLoRaPriority, buf, 0) == pdTRUE;
}

OperatingMode readMode() {
  OperatingMode m = OperatingMode::AUTO;
  if (g_semMode && xSemaphoreTake(g_semMode, pdMS_TO_TICKS(20)) == pdTRUE) {
    m = g_currentMode;
    xSemaphoreGive(g_semMode);
  }
  return m;
}

DriveCmd readCmd(uint32_t& lastCmdMs) {
  DriveCmd c = DriveCmd::STOP;
  lastCmdMs = 0;
  if (g_semCmd && xSemaphoreTake(g_semCmd, pdMS_TO_TICKS(20)) == pdTRUE) {
    c = g_currentCmd;
    lastCmdMs = g_lastMqttCmdMs;
    xSemaphoreGive(g_semCmd);
  }
  return c;
}

uint8_t readSafetyIndex() {
  uint8_t s = 9;
  if (g_semSafety && xSemaphoreTake(g_semSafety, pdMS_TO_TICKS(20)) == pdTRUE) {
    s = g_safetyIndex;
    xSemaphoreGive(g_semSafety);
  }
  return s;
}

void readScan(uint16_t out[RADAR_STEPS]) {
  if (g_semScan && xSemaphoreTake(g_semScan, pdMS_TO_TICKS(20)) == pdTRUE) {
    memcpy(out, g_scanArray, sizeof(uint16_t) * RADAR_STEPS);
    xSemaphoreGive(g_semScan);
  } else {
    memset(out, 0, sizeof(uint16_t) * RADAR_STEPS);
  }
}

void writeMode(OperatingMode m) {
  if (!g_semMode) return;
  if (xSemaphoreTake(g_semMode, pdMS_TO_TICKS(20)) == pdTRUE) {
    g_currentMode = m;
    xSemaphoreGive(g_semMode);
  }
}

void writeState(VehicleState s) {
  if (!g_semState) return;
  if (xSemaphoreTake(g_semState, pdMS_TO_TICKS(20)) == pdTRUE) {
    g_vehicleState = s;
    xSemaphoreGive(g_semState);
  }
}

VehicleState readState() {
  VehicleState s = VehicleState::STATE_BOOT;
  if (g_semState && xSemaphoreTake(g_semState, pdMS_TO_TICKS(20)) == pdTRUE) {
    s = g_vehicleState;
    xSemaphoreGive(g_semState);
  }
  return s;
}

void writeEnv(float t, float h) {
  if (!g_semEnv) return;
  if (xSemaphoreTake(g_semEnv, pdMS_TO_TICKS(20)) == pdTRUE) {
    g_tempC = t;
    g_humPct = h;
    xSemaphoreGive(g_semEnv);
  }
}

void readEnv(float& t, float& h) {
  t = NAN;
  h = NAN;
  if (g_semEnv && xSemaphoreTake(g_semEnv, pdMS_TO_TICKS(20)) == pdTRUE) {
    t = g_tempC;
    h = g_humPct;
    xSemaphoreGive(g_semEnv);
  }
}

}  // namespace

// =========================
// taskMQTT (Core 0)
// =========================
void taskMQTT(void* pv) {
  (void)pv;
  for (;;) {
    Network::mqttLoop(millis());
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// =========================
// taskLoRa (Core 0)
// =========================
void taskLoRa(void* pv) {
  (void)pv;

  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastTelemMs = 0;

  for (;;) {
    // Handle priority alerts first
    if (g_qLoRaPriority) {
      char buf[96] = {0};
      while (xQueueReceive(g_qLoRaPriority, buf, 0) == pdTRUE) {
        Network::loraSendAlert(buf);
      }
    }

    const uint32_t nowMs = millis();
    if ((nowMs - lastTelemMs) >= LORA_TELEMETRY_PERIOD_MS) {
      lastTelemMs = nowMs;

      float t = NAN, h = NAN;
      readEnv(t, h);
      const uint8_t s = readSafetyIndex();
      const VehicleState st = readState();

      // If env isn't ready yet, send NAN as 0.0 to keep payload stable.
      if (!isfinite(t)) t = 0.0f;
      if (!isfinite(h)) h = 0.0f;

      Network::loraSendTelemetry(t, h, s, st);
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(25));
  }
}

// =========================
// taskSensors (Core 1)
// =========================
void taskSensors(void* pv) {
  (void)pv;

  DHT dht(PIN_DHT, DHT11);
  dht.begin();

  LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ESP32 4WD");
  lcd.setCursor(0, 1);
  lcd.print("BOOTING...");

  uint32_t lastDhtMs = 0;
  uint32_t lastLcdMs = 0;
  float tempC = NAN;
  float humPct = NAN;

  const TickType_t tick = pdMS_TO_TICKS(SENSORS_TICK_MS);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    const uint32_t nowMs = millis();

    if ((nowMs - lastDhtMs) >= DHT_SAMPLE_PERIOD_MS) {
      lastDhtMs = nowMs;
      const float t = dht.readTemperature();
      const float h = dht.readHumidity();
      if (isfinite(t) && isfinite(h)) {
        tempC = t;
        humPct = h;
        writeEnv(tempC, humPct);
      }
    }

    if ((nowMs - lastLcdMs) >= LCD_REFRESH_MS) {
      lastLcdMs = nowMs;

      float t = NAN, h = NAN;
      readEnv(t, h);
      const uint8_t s = readSafetyIndex();
      const VehicleState st = readState();

      uint16_t scan[RADAR_STEPS] = {0};
      readScan(scan);
      uint16_t front = scan[3];

      lcd.clear();

      // Line 1: T/H
      lcd.setCursor(0, 0);
      lcd.print("T:");
      if (isfinite(t)) {
        lcd.print(static_cast<int>(t));
        lcd.print("C ");
      } else {
        lcd.print("--C ");
      }
      lcd.print("H:");
      if (isfinite(h)) {
        lcd.print(static_cast<int>(h));
        lcd.print("%");
      } else {
        lcd.print("--%");
      }

      // Line 2: S, State, Dist
      lcd.setCursor(0, 1);
      lcd.print("S:");
      lcd.print(s);
      lcd.print(" ");
      // 6 chars budget for state
      switch (st) {
        case VehicleState::STATE_FORWARD:
          lcd.print("FWD");
          break;
        case VehicleState::STATE_AVOID:
          lcd.print("AVOID");
          break;
        case VehicleState::STATE_TRAPPED:
          lcd.print("TRAP");
          break;
        case VehicleState::STATE_FORCE_STOP:
          lcd.print("STOP");
          break;
        case VehicleState::STATE_MANUAL:
          lcd.print("MAN");
          break;
        default:
          lcd.print("IDLE");
          break;
      }
      lcd.print(" D:");
      if (front == 0) {
        lcd.print("--");
      } else {
        lcd.print(front);
      }
      lcd.print("cm");
    }

    vTaskDelayUntil(&lastWake, tick);
  }
}

// =========================
// taskMotorFSM (Core 1)
// =========================
void taskMotorFSM(void* pv) {
  (void)pv;

  auto scoreDistance = [](uint16_t cm) -> uint8_t {
    if (cm < DIST_BUCKET_NEAR_CM) return 1;
    if (cm <= DIST_BUCKET_MID_CM) return 5;
    return 9;
  };
  auto scoreTemp = [](float c) -> uint8_t {
    if (c > TEMP_BUCKET_HOT_C) return 1;
    if (c >= TEMP_BUCKET_WARM_C) return 5;
    return 9;
  };
  auto scoreHum = [](float h) -> uint8_t {
    if (h > HUM_BUCKET_HIGH_PCT || h < HUM_BUCKET_LOW_PCT) return 5;
    return 9;
  };
  auto calcSafetyIndexLocal = [&](uint16_t frontCm, float t, float h) -> uint8_t {
    const uint8_t sD = scoreDistance(frontCm);
    const uint8_t sT = scoreTemp(t);
    const uint8_t sH = scoreHum(h);
    const float S = 0.6f * sD + 0.3f * sT + 0.1f * sH;
    int si = static_cast<int>(floorf(S));
    if (si < 0) si = 0;
    if (si > 9) si = 9;
    return static_cast<uint8_t>(si);
  };

  for (;;) {
    const uint32_t nowMs = millis();

    const OperatingMode mode = readMode();
    uint32_t lastCmdMs = 0;
    const DriveCmd manualCmd = readCmd(lastCmdMs);
    if (lastCmdMs == 0) lastCmdMs = nowMs;

    // =========================
    // MANUAL (unchanged behavior)
    // =========================
    if (mode == OperatingMode::MANUAL) {
      if ((nowMs - lastCmdMs) > MANUAL_AUTO_RETURN_MS) {
        writeMode(OperatingMode::AUTO);
        (void)queueAlert("INFO:MANUAL->AUTO_TIMEOUT");
        HAL::motorStop();
        vTaskDelay(pdMS_TO_TICKS(20));
        continue;
      }

      writeState(VehicleState::STATE_MANUAL);

      if ((nowMs - lastCmdMs) > MANUAL_DEADMAN_MS) {
        HAL::motorStop();
        vTaskDelay(pdMS_TO_TICKS(20));
        continue;
      }

      switch (manualCmd) {
        case DriveCmd::FORWARD:
          HAL::motorApply(DriveCmd::FORWARD, pwmPctToDuty(PWM_MANUAL_FWD_PCT));
          break;
        case DriveCmd::BACKWARD:
          HAL::motorApply(DriveCmd::BACKWARD, pwmPctToDuty(PWM_REVERSE_PCT));
          break;
        case DriveCmd::LEFT:
        case DriveCmd::RIGHT:
          HAL::motorApply(manualCmd, pwmPctToDuty(PWM_MANUAL_TURN_PCT));
          break;
        case DriveCmd::STOP:
        default:
          HAL::motorStop();
          break;
      }

      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // =========================
    // AUTO: Stop-and-Scan logic
    // =========================

    // Step 1: ensure servo at 90° but don't block 200ms while cruising forward.
    // The servo is normally already centered; only yield briefly for RTOS scheduling.
    HAL::servoWriteAngle(90);
    vTaskDelay(pdMS_TO_TICKS(20));
    const uint16_t frontDistance = HAL::ultrasonicReadCm();

    if (g_semScan && xSemaphoreTake(g_semScan, pdMS_TO_TICKS(20)) == pdTRUE) {
      g_scanArray[3] = frontDistance;
      xSemaphoreGive(g_semScan);
    }

    // Safety index update
    float tempC = NAN, humPct = NAN;
    readEnv(tempC, humPct);
    if (!isfinite(tempC)) tempC = 25.0f;
    if (!isfinite(humPct)) humPct = 50.0f;
    // Treat 0 as "invalid / dead-zone" => worst-case distance for safety.
    const uint16_t distForSafety = (frontDistance == 0) ? 1 : frontDistance;
    const uint8_t safety = calcSafetyIndexLocal(distForSafety, tempC, humPct);
    if (g_semSafety && xSemaphoreTake(g_semSafety, pdMS_TO_TICKS(20)) == pdTRUE) {
      g_safetyIndex = safety;
      xSemaphoreGive(g_semSafety);
    }

    if (safety <= SAFETY_FORCE_STOP_THRESHOLD) {
      writeState(VehicleState::STATE_FORCE_STOP);
      HAL::motorStop();
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // Only go forward when we are confidently clear.
    // If frontDistance == 0 (dead-zone/timeout), treat as risk and enter AVOID.
    if (frontDistance > AVOID_TRIGGER_CM) {
      writeState(VehicleState::STATE_FORWARD);
      HAL::motorApply(DriveCmd::FORWARD, pwmPctToDuty(PWM_AUTO_FWD_PCT));
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // Step 2: obstacle detected (distance <= threshold OR invalid 0 due to dead-zone)
    writeState(VehicleState::STATE_AVOID);

    // a. Stop
    HAL::motorStop();
    vTaskDelay(pdMS_TO_TICKS(50));

    // b. Reverse for 300ms, then stop
    HAL::motorApply(DriveCmd::BACKWARD, pwmPctToDuty(PWM_REVERSE_PCT));
    vTaskDelay(pdMS_TO_TICKS(300));
    HAL::motorStop();
    vTaskDelay(pdMS_TO_TICKS(50));

    // c. Servo to 180°, wait 400ms, read left
    HAL::servoWriteAngle(180);
    vTaskDelay(pdMS_TO_TICKS(400));
    uint16_t leftDistance = HAL::ultrasonicReadCm();

    // d. Servo to 0°, wait 400ms, read right
    HAL::servoWriteAngle(0);
    vTaskDelay(pdMS_TO_TICKS(400));
    uint16_t rightDistance = HAL::ultrasonicReadCm();

    if (g_semScan && xSemaphoreTake(g_semScan, pdMS_TO_TICKS(20)) == pdTRUE) {
      // 0°=RIGHT idx 0, 180°=LEFT idx 6
      g_scanArray[0] = rightDistance;
      g_scanArray[6] = leftDistance;
      xSemaphoreGive(g_semScan);
    }

    // e. Servo back to 90°, wait 200ms
    HAL::servoWriteAngle(90);
    vTaskDelay(pdMS_TO_TICKS(200));

    // f. Compare and hard-turn for ESCAPE_HARD_TURN_MS
    const uint16_t leftCmp = (leftDistance == 0) ? 999 : leftDistance;
    const uint16_t rightCmp = (rightDistance == 0) ? 999 : rightDistance;
    const DriveCmd turnCmd = (leftCmp >= rightCmp) ? DriveCmd::LEFT : DriveCmd::RIGHT;
    HAL::motorApply(turnCmd, pwmPctToDuty(PWM_AUTO_TURN_PCT));
    vTaskDelay(pdMS_TO_TICKS(ESCAPE_HARD_TURN_MS));

    // g. Stop and loop
    HAL::motorStop();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
