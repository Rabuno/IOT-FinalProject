#pragma once

#include <Arduino.h>

#include "../../include/config.h"

namespace HAL {

// Call once from setup().
void initPins();
void initPwm();

// =========================
// Motor Control (L298N)
// =========================
// duty: 0..255 for 8-bit resolution
void motorApply(DriveCmd cmd, uint8_t duty);
void motorStop();

// =========================
// Servo Control (Radar)
// =========================
// angleDeg: 0..180
void servoWriteAngle(uint8_t angleDeg);

// =========================
// Ultrasonic (SRF05)
// =========================
// Returns distance in cm; 0 indicates timeout/invalid reading.
uint16_t ultrasonicReadCm(uint32_t timeoutUs = US_ECHO_TIMEOUT_US);

}  // namespace HAL
