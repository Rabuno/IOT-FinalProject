#include "Radar.h"

#include "HAL.h"

namespace Radar {

namespace {
uint8_t applyServoOffset(uint8_t angleDeg) {
  return clampServoDeg(static_cast<int>(angleDeg) + SERVO_CENTER_OFFSET_DEG);
}
}  // namespace

void Scanner::startScan(uint32_t nowMs) {
  idx_ = 0;
  phase_ = Phase::MOVE;
  phaseStartMs_ = nowMs;
  active_ = true;
  complete_ = false;
  scheduleMove_(nowMs);
}

void Scanner::scheduleMove_(uint32_t nowMs) {
  phase_ = Phase::MOVE;
  phaseStartMs_ = nowMs;
  HAL::servoWriteAngle(applyServoOffset(RADAR_ANGLES_DEG[idx_]));
  scheduleSettle_(nowMs);
}

void Scanner::scheduleSettle_(uint32_t nowMs) {
  phase_ = Phase::SETTLE;
  phaseStartMs_ = nowMs;
}

bool Scanner::tick(uint32_t nowMs, uint16_t scanOut[RADAR_STEPS]) {
  if (!active_) return false;

  switch (phase_) {
    case Phase::MOVE:
      // MOVE immediately schedules SETTLE; this phase exists for clarity/future expansion.
      scheduleSettle_(nowMs);
      return false;

    case Phase::SETTLE:
      if ((nowMs - phaseStartMs_) < RADAR_SETTLE_MS) return false;
      phase_ = Phase::READ;
      phaseStartMs_ = nowMs;
      [[fallthrough]];

    case Phase::READ: {
      const uint16_t cm = HAL::ultrasonicReadCm();
      scanOut[idx_] = cm;

      // Advance to next discrete angle (0 -> 180), then stop.
      idx_++;
      if (idx_ >= RADAR_STEPS) {
        active_ = false;
        complete_ = true;
        // Return servo to forward (90°) at the end of scan.
        HAL::servoWriteAngle(SERVO_CENTER_DEG);
      } else {
        scheduleMove_(nowMs);
      }
      return true;
    }
  }

  return false;
}

}  // namespace Radar
