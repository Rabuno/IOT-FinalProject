#pragma once

#include <Arduino.h>

#include "config.h"

// This header contains pure decision logic (no direct I/O).
// It is intended to be used by taskMotorFSM and other tasks.

enum class AlertCode : uint8_t {
  NONE = 0,
  FORCE_STOP,
  TRAPPED,
  KINEMATIC_STALL,
  AVOID_TIMEOUT,
};

struct SafetyScores {
  uint8_t sD;
  uint8_t sT;
  uint8_t sH;
};

static inline uint8_t scoreDistanceCm(uint16_t cm) {
  if (cm < DIST_BUCKET_NEAR_CM) return 1;
  if (cm <= DIST_BUCKET_MID_CM) return 5;
  return 9;
}

static inline uint8_t scoreTempC(float c) {
  if (c > TEMP_BUCKET_HOT_C) return 1;
  if (c >= TEMP_BUCKET_WARM_C) return 5;
  return 9;
}

static inline uint8_t scoreHumPct(float h) {
  if (h > HUM_BUCKET_HIGH_PCT || h < HUM_BUCKET_LOW_PCT) return 5;
  return 9;
}

static inline SafetyScores calcSafetyScores(uint16_t distanceCm, float tempC, float humPct) {
  SafetyScores out{};
  out.sD = scoreDistanceCm(distanceCm);
  out.sT = scoreTempC(tempC);
  out.sH = scoreHumPct(humPct);
  return out;
}

static inline uint8_t calcSafetyIndex(const SafetyScores& s) {
  // S = floor(0.6*sD + 0.3*sT + 0.1*sH)
  const float S = 0.6f * s.sD + 0.3f * s.sT + 0.1f * s.sH;
  int si = static_cast<int>(floorf(S));
  if (si < 0) si = 0;
  if (si > 9) si = 9;
  return static_cast<uint8_t>(si);
}

struct GapVector {
  // -1 => left, 0 => forward, +1 => right
  int8_t dir;
  uint8_t centerIdx;
  uint16_t clearanceCm;
};

static inline GapVector findMaxClearanceGap(const uint16_t scan[RADAR_STEPS],
                                            uint16_t minClearanceCm = FORWARD_RESUME_CM) {
  // Find the widest contiguous region that satisfies minClearanceCm.
  // Tie-breaker: higher sum clearance; then center closest to straight (idx 3).
  struct Region {
    uint8_t start;
    uint8_t end;  // inclusive
    uint32_t sum;
  };

  Region best{0, 0, 0};
  bool hasBest = false;

  uint8_t i = 0;
  while (i < RADAR_STEPS) {
    while (i < RADAR_STEPS && scan[i] < minClearanceCm) i++;
    if (i >= RADAR_STEPS) break;

    const uint8_t start = i;
    uint32_t sum = 0;
    while (i < RADAR_STEPS && scan[i] >= minClearanceCm) {
      sum += scan[i];
      i++;
    }
    const uint8_t end = static_cast<uint8_t>(i - 1);

    const uint8_t width = static_cast<uint8_t>(end - start + 1);
    const uint8_t bestWidth = static_cast<uint8_t>(best.end - best.start + 1);
    const uint8_t center = static_cast<uint8_t>((start + end) / 2);
    const uint8_t bestCenter = static_cast<uint8_t>((best.start + best.end) / 2);

    const uint8_t centerDist = static_cast<uint8_t>(abs(static_cast<int>(center) - 3));
    const uint8_t bestCenterDist = static_cast<uint8_t>(abs(static_cast<int>(bestCenter) - 3));

    bool take = false;
    if (!hasBest) {
      take = true;
    } else if (width > bestWidth) {
      take = true;
    } else if (width == bestWidth && sum > best.sum) {
      take = true;
    } else if (width == bestWidth && sum == best.sum && centerDist < bestCenterDist) {
      take = true;
    }

    if (take) {
      best = Region{start, end, sum};
      hasBest = true;
    }
  }

  if (!hasBest) {
    // No region meets minClearanceCm: pick the single best point.
    uint8_t bestIdx = 0;
    uint16_t bestVal = scan[0];
    for (uint8_t k = 1; k < RADAR_STEPS; k++) {
      const uint16_t v = scan[k];
      if (v > bestVal) {
        bestVal = v;
        bestIdx = k;
      } else if (v == bestVal) {
        // Prefer closer to straight (idx 3)
        const uint8_t d1 = static_cast<uint8_t>(abs(static_cast<int>(k) - 3));
        const uint8_t d0 = static_cast<uint8_t>(abs(static_cast<int>(bestIdx) - 3));
        if (d1 < d0) bestIdx = k;
      }
    }
    // Mechanical convention: servo 0° points RIGHT, 180° points LEFT.
    // Therefore low indices are RIGHT, high indices are LEFT.
    const int8_t dir = (bestIdx < 3) ? +1 : (bestIdx > 3) ? -1 : 0;
    return GapVector{dir, bestIdx, bestVal};
  }

  const uint8_t centerIdx = static_cast<uint8_t>((best.start + best.end) / 2);
  const uint16_t clearance = scan[centerIdx];
  const int8_t dir = (centerIdx < 3) ? +1 : (centerIdx > 3) ? -1 : 0;
  return GapVector{dir, centerIdx, clearance};
}

class AntiTrapMonitor {
 public:
  void reset() {
    count_ = 0;
    windowStartMs_ = 0;
    lastFrom_ = VehicleState::STATE_BOOT;
    lastTo_ = VehicleState::STATE_BOOT;
  }

  void recordTransition(VehicleState from, VehicleState to, uint32_t nowMs) {
    const bool isFwdAvoidPair =
        (from == VehicleState::STATE_FORWARD && to == VehicleState::STATE_AVOID) ||
        (from == VehicleState::STATE_AVOID && to == VehicleState::STATE_FORWARD);

    if (!isFwdAvoidPair) {
      lastFrom_ = from;
      lastTo_ = to;
      return;
    }

    if (count_ == 0) {
      windowStartMs_ = nowMs;
      count_ = 1;
    } else {
      if ((nowMs - windowStartMs_) <= ANTI_TRAP_WINDOW_MS) {
        count_++;
      } else {
        windowStartMs_ = nowMs;
        count_ = 1;
      }
    }

    lastFrom_ = from;
    lastTo_ = to;
  }

  bool isTrapped(uint32_t nowMs) const {
    if (count_ < ANTI_TRAP_TRANSITION_LIMIT) return false;
    return (nowMs - windowStartMs_) <= ANTI_TRAP_WINDOW_MS;
  }

 private:
  uint8_t count_ = 0;
  uint32_t windowStartMs_ = 0;
  VehicleState lastFrom_ = VehicleState::STATE_BOOT;
  VehicleState lastTo_ = VehicleState::STATE_BOOT;
};

class TrapEscalationMonitor {
 public:
  void reset() {
    trappedCount_ = 0;
    firstTrapMs_ = 0;
  }

  void recordTrap(uint32_t nowMs) {
    if (trappedCount_ == 0) {
      trappedCount_ = 1;
      firstTrapMs_ = nowMs;
      return;
    }

    if ((nowMs - firstTrapMs_) <= TRAPPED_TWICE_WINDOW_MS) {
      trappedCount_++;
    } else {
      trappedCount_ = 1;
      firstTrapMs_ = nowMs;
    }
  }

  bool trappedTwiceInWindow(uint32_t nowMs) const {
    if (trappedCount_ < TRAPPED_TWICE_LIMIT) return false;
    return (nowMs - firstTrapMs_) <= TRAPPED_TWICE_WINDOW_MS;
  }

 private:
  uint8_t trappedCount_ = 0;
  uint32_t firstTrapMs_ = 0;
};

class KinematicStallDetector {
 public:
  void reset() {
    forwardStartMs_ = 0;
    distanceAtStartCm_ = 0;
    validAtStart_ = false;
    active_ = false;
  }

  void onForwardStart(uint16_t distanceCm, bool valid, uint32_t nowMs) {
    active_ = true;
    forwardStartMs_ = nowMs;
    distanceAtStartCm_ = distanceCm;
    validAtStart_ = valid;
  }

  void onNotForward() { active_ = false; }

  bool isStalled(uint16_t currentDistanceCm, bool valid, uint32_t nowMs) const {
    if (!active_) return false;
    if ((nowMs - forwardStartMs_) < KIN_STALL_WINDOW_MS) return false;
    if (!validAtStart_ || !valid) return false;
    if (distanceAtStartCm_ == 0 || currentDistanceCm == 0) return false;
    if (distanceAtStartCm_ > KIN_STALL_MAX_DISTANCE_CM) return false;
    if (currentDistanceCm > KIN_STALL_MAX_DISTANCE_CM) return false;

    // Only consider stall when an obstacle is within a reasonable "tracking" range.
    // In open space (no obstacle in sight), distance can remain constant and should not trigger.
    if (distanceAtStartCm_ > KIN_STALL_ARM_DISTANCE_CM) return false;
    if (currentDistanceCm > KIN_STALL_ARM_DISTANCE_CM) return false;

    const uint16_t d0 = distanceAtStartCm_;
    const uint16_t d1 = currentDistanceCm;
    const uint16_t delta = (d0 > d1) ? static_cast<uint16_t>(d0 - d1) : static_cast<uint16_t>(d1 - d0);
    return delta < KIN_STALL_MIN_DELTA_CM;
  }

 private:
  uint32_t forwardStartMs_ = 0;
  uint16_t distanceAtStartCm_ = 0;
  bool validAtStart_ = false;
  bool active_ = false;
};

class MotorFSM {
 public:
  struct Inputs {
    OperatingMode mode;
    DriveCmd manualCmd;
    uint8_t safetyIndex;
    uint32_t lastMqttCmdMs;
    const uint16_t* scanArray;  // size RADAR_STEPS
  };

  struct Outputs {
    VehicleState state;
    DriveCmd motorCmd;
    AlertCode alert;
    bool requestManualMode;  // set true when trapped twice in 15s
  };

  void reset(uint32_t nowMs) {
    state_ = VehicleState::STATE_BOOT;
    escapePhase_ = EscapePhase::NONE;
    escapeStartMs_ = nowMs;
    escapeTurnCmd_ = DriveCmd::RIGHT;
    escapeTurnToggle_ = false;
    antiTrap_.reset();
    trapEscalation_.reset();
    kinStall_.reset();
    lastStateChangeMs_ = nowMs;
    avoidEnterMs_ = nowMs;
  }

  Outputs tick(const Inputs& in, uint32_t nowMs) {
    Outputs out{};
    out.alert = AlertCode::NONE;
    out.requestManualMode = false;

    // Global veto rule: safety index forces hard stop regardless of mode/command.
    if (in.safetyIndex <= SAFETY_FORCE_STOP_THRESHOLD) {
      transitionTo(VehicleState::STATE_FORCE_STOP, nowMs);
      out.state = state_;
      out.motorCmd = DriveCmd::STOP;
      out.alert = AlertCode::FORCE_STOP;
      return out;
    }

    // Manual mode: enforce deadman switch.
    if (in.mode == OperatingMode::MANUAL) {
      transitionTo(VehicleState::STATE_MANUAL, nowMs);
      out.state = state_;

      if ((nowMs - in.lastMqttCmdMs) > MANUAL_DEADMAN_MS) {
        out.motorCmd = DriveCmd::STOP;
      } else {
        out.motorCmd = sanitizeManualCmd(in.manualCmd);
      }

      kinStall_.onNotForward();
      escapePhase_ = EscapePhase::NONE;
      return out;
    }

    // Auto mode decision-making.
    const uint16_t frontRaw = (in.scanArray != nullptr) ? in.scanArray[3] : 0;
    const bool frontValid = (frontRaw > 0);
    const uint16_t frontCm = frontValid ? frontRaw : 500;
    bool obstacleNear = frontCm > 0 && frontCm < AVOID_TRIGGER_CM;
    // If we have a recent scan (60/90/120 deg), use it as a safety cone to prevent side-swipe.
    if (in.scanArray != nullptr) {
      const uint16_t a = in.scanArray[2];
      const uint16_t b = in.scanArray[3];
      const uint16_t c = in.scanArray[4];
      const bool coneNear = ((a > 0 && a < AVOID_TRIGGER_CM) || (b > 0 && b < AVOID_TRIGGER_CM) ||
                             (c > 0 && c < AVOID_TRIGGER_CM));
      obstacleNear = obstacleNear || coneNear;
    }
    const bool coneClear = (in.scanArray != nullptr) &&
                           (in.scanArray[2] > FORWARD_RESUME_CM) && (in.scanArray[3] > FORWARD_RESUME_CM) &&
                           (in.scanArray[4] > FORWARD_RESUME_CM);

    switch (state_) {
      case VehicleState::STATE_BOOT:
      case VehicleState::STATE_IDLE:
      case VehicleState::STATE_MANUAL:
        transitionTo(VehicleState::STATE_FORWARD, nowMs);
        kinStall_.onForwardStart(frontRaw, frontValid, nowMs);
        break;

      case VehicleState::STATE_FORWARD: {
        if (obstacleNear) {
          transitionTo(VehicleState::STATE_AVOID, nowMs);
          kinStall_.onNotForward();
          break;
        }

        // Kinematic stall detection in forward drive.
        if (kinStall_.isStalled(frontRaw, frontValid, nowMs)) {
          setEscapeTurnFromScan_(in.scanArray);
          transitionTo(VehicleState::STATE_TRAPPED, nowMs);
          out.alert = AlertCode::KINEMATIC_STALL;
          trapEscalation_.recordTrap(nowMs);
          break;
        }
        break;
      }

      case VehicleState::STATE_AVOID: {
        if (coneClear) {
          transitionTo(VehicleState::STATE_FORWARD, nowMs);
          kinStall_.onForwardStart(frontRaw, frontValid, nowMs);
        } else if ((nowMs - avoidEnterMs_) > AVOID_TIMEOUT_MS) {
          setEscapeTurnFromScan_(in.scanArray);
          transitionTo(VehicleState::STATE_TRAPPED, nowMs);
          out.alert = AlertCode::AVOID_TIMEOUT;
          trapEscalation_.recordTrap(nowMs);
        } else if (antiTrap_.isTrapped(nowMs)) {
          setEscapeTurnFromScan_(in.scanArray);
          transitionTo(VehicleState::STATE_TRAPPED, nowMs);
          out.alert = AlertCode::TRAPPED;
          trapEscalation_.recordTrap(nowMs);
        }
        break;
      }

      case VehicleState::STATE_TRAPPED: {
        // After the escape maneuver completes, attempt forward again.
        if (escapePhase_ == EscapePhase::NONE && (nowMs - lastStateChangeMs_) > (ESCAPE_REVERSE_MS + ESCAPE_HARD_TURN_MS)) {
          transitionTo(VehicleState::STATE_FORWARD, nowMs);
          kinStall_.onForwardStart(frontRaw, frontValid, nowMs);
          break;
        }

        // Escalate to manual after repeated trap events.
        if (trapEscalation_.trappedTwiceInWindow(nowMs)) {
          out.requestManualMode = true;
          transitionTo(VehicleState::STATE_MANUAL, nowMs);
          kinStall_.onNotForward();
          escapePhase_ = EscapePhase::NONE;
        }
        break;
      }

      case VehicleState::STATE_FORCE_STOP:
        // Can only exit force stop when safety index recovers; handled above by not re-entering.
        transitionTo(VehicleState::STATE_FORWARD, nowMs);
        kinStall_.onForwardStart(frontRaw, frontValid, nowMs);
        break;
    }

    // Motor command synthesis based on current state.
    out.state = state_;
    out.motorCmd = decideCmdForState(state_, in.scanArray, frontCm, nowMs);
    return out;
  }

 private:
  enum class EscapePhase : uint8_t { NONE = 0, REVERSE, HARD_TURN };

  static DriveCmd sanitizeManualCmd(DriveCmd cmd) {
    switch (cmd) {
      case DriveCmd::STOP:
      case DriveCmd::FORWARD:
      case DriveCmd::BACKWARD:
      case DriveCmd::LEFT:
      case DriveCmd::RIGHT:
        return cmd;
    }
    return DriveCmd::STOP;
  }

  void transitionTo(VehicleState next, uint32_t nowMs) {
    if (next == state_) return;
    antiTrap_.recordTransition(state_, next, nowMs);
    state_ = next;
    lastStateChangeMs_ = nowMs;
    if (state_ == VehicleState::STATE_AVOID) avoidEnterMs_ = nowMs;

    if (state_ != VehicleState::STATE_TRAPPED) {
      escapePhase_ = EscapePhase::NONE;
    } else {
      escapePhase_ = EscapePhase::REVERSE;
      escapeStartMs_ = nowMs;
    }
  }

  DriveCmd decideCmdForState(VehicleState st,
                             const uint16_t* scanArray,
                             uint16_t frontCm,
                             uint32_t nowMs) {
    switch (st) {
      case VehicleState::STATE_FORCE_STOP:
        return DriveCmd::STOP;

      case VehicleState::STATE_TRAPPED:
        return decideEscapeCmd(nowMs);

      case VehicleState::STATE_AVOID: {
        if (scanArray == nullptr) return DriveCmd::STOP;
        const GapVector v = findMaxClearanceGap(scanArray, AVOID_TRIGGER_CM);

        // Mechanical convention: 0° is RIGHT, 180° is LEFT.
        // idx: 0..6 -> angles: 0,30,60,90,120,150,180 (right..left)
        if (v.centerIdx <= 2) return DriveCmd::RIGHT;  // point-turn right
        if (v.centerIdx == 3) return DriveCmd::STOP;   // hold until cone clears
        return DriveCmd::LEFT;                         // point-turn left (idx 4/5/6)
      }

      case VehicleState::STATE_FORWARD:
      case VehicleState::STATE_IDLE:
      case VehicleState::STATE_BOOT:
      case VehicleState::STATE_MANUAL:
      default:
        return DriveCmd::FORWARD;
    }
  }

  DriveCmd decideEscapeCmd(uint32_t nowMs) {
    if (escapePhase_ == EscapePhase::NONE) {
      escapePhase_ = EscapePhase::REVERSE;
      escapeStartMs_ = nowMs;
    }

    const uint32_t elapsed = nowMs - escapeStartMs_;
    if (escapePhase_ == EscapePhase::REVERSE) {
      if (elapsed >= ESCAPE_REVERSE_MS) {
        escapePhase_ = EscapePhase::HARD_TURN;
        escapeStartMs_ = nowMs;
      }
      return DriveCmd::BACKWARD;
    }

    if (escapePhase_ == EscapePhase::HARD_TURN) {
      if (elapsed >= ESCAPE_HARD_TURN_MS) {
        escapePhase_ = EscapePhase::NONE;
      }
      return escapeTurnCmd_;
    }

    return DriveCmd::STOP;
  }

  VehicleState state_ = VehicleState::STATE_BOOT;
  uint32_t lastStateChangeMs_ = 0;
  uint32_t avoidEnterMs_ = 0;

  // Escape maneuver internal state
  EscapePhase escapePhase_ = EscapePhase::NONE;
  uint32_t escapeStartMs_ = 0;
  DriveCmd escapeTurnCmd_ = DriveCmd::RIGHT;
  bool escapeTurnToggle_ = false;

  // Monitors
  AntiTrapMonitor antiTrap_{};
  TrapEscalationMonitor trapEscalation_{};
  KinematicStallDetector kinStall_{};

  void setEscapeTurnFromScan_(const uint16_t* scanArray) {
    // Mechanical convention: indices 0..2 are RIGHT side, 4..6 are LEFT side.
    if (scanArray == nullptr) {
      // No scan info: alternate to avoid getting stuck in a biased direction.
      escapeTurnToggle_ = !escapeTurnToggle_;
      escapeTurnCmd_ = escapeTurnToggle_ ? DriveCmd::LEFT : DriveCmd::RIGHT;
      return;
    }

    auto safe = [](uint16_t v) -> uint16_t { return (v == 0) ? 0 : v; };
    const uint16_t rightMax = max(safe(scanArray[0]), max(safe(scanArray[1]), safe(scanArray[2])));
    const uint16_t leftMax = max(safe(scanArray[4]), max(safe(scanArray[5]), safe(scanArray[6])));

    if (rightMax == 0 && leftMax == 0) {
      escapeTurnToggle_ = !escapeTurnToggle_;
      escapeTurnCmd_ = escapeTurnToggle_ ? DriveCmd::LEFT : DriveCmd::RIGHT;
      return;
    }
    escapeTurnCmd_ = (rightMax >= leftMax) ? DriveCmd::RIGHT : DriveCmd::LEFT;
  }
};
