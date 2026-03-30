#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#include "config.h"

// =========================
// Shared State (Protected)
// =========================
// Strict IPC: access MUST be guarded by the corresponding semaphore.
extern SemaphoreHandle_t g_semMode;
extern SemaphoreHandle_t g_semCmd;
extern SemaphoreHandle_t g_semSafety;
extern SemaphoreHandle_t g_semScan;
extern SemaphoreHandle_t g_semEnv;
extern SemaphoreHandle_t g_semState;

extern OperatingMode g_currentMode;
extern DriveCmd g_currentCmd;
extern uint8_t g_safetyIndex;
extern uint16_t g_scanArray[RADAR_STEPS];
extern float g_tempC;
extern float g_humPct;
extern VehicleState g_vehicleState;

// Manual-mode deadman support (ms since boot). Guarded by g_semCmd in this project.
extern uint32_t g_lastMqttCmdMs;

// =========================
// Queues (Optional IPC)
// =========================
// Priority LoRa alerts (e.g., trapped / force-stop). Payload is a null-terminated C string.
extern QueueHandle_t g_qLoRaPriority;
