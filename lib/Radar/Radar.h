#pragma once

#include <Arduino.h>

#include "../../include/config.h"

namespace Radar {

// Non-blocking sweep controller for a servo-mounted ultrasonic sensor.
// Controlled by FSM: startScan() arms a single 0°->180° scan (7 discrete angles).
// Call tick() frequently ONLY while scanning is active.
class Scanner {
 public:
  void startScan(uint32_t nowMs);
  bool isActive() const { return active_; }
  bool isComplete() const { return complete_; }
  void reset() {
    active_ = false;
    complete_ = false;
    idx_ = 0;
    phase_ = Phase::MOVE;
    phaseStartMs_ = 0;
  }

  // Returns true when a new reading was written into scanOut at the current index.
  // scanOut must point to an array of size RADAR_STEPS.
  bool tick(uint32_t nowMs, uint16_t scanOut[RADAR_STEPS]);

  uint8_t currentIndex() const { return idx_; }
  uint8_t currentAngleDeg() const { return RADAR_ANGLES_DEG[idx_]; }

 private:
  enum class Phase : uint8_t { MOVE = 0, SETTLE, READ };

  void scheduleMove_(uint32_t nowMs);
  void scheduleSettle_(uint32_t nowMs);

  Phase phase_ = Phase::MOVE;
  uint8_t idx_ = 0;
  uint32_t phaseStartMs_ = 0;
  bool active_ = false;
  bool complete_ = false;
};

}  // namespace Radar
