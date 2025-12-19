// ============================================================================
// Safety Monitor Implementation
// ============================================================================

#include "safety/safety_monitor.h"

SafetyMonitor::SafetyMonitor(SteeringController &steering,
                             DriveController &drive)
    : steering_(steering), drive_(drive), cmdReceived_(false), lastCmdTime_(0),
      timeoutActive_(false), gracePeriod_(false), graceStartTime_(0) {}

void SafetyMonitor::update() {
  unsigned long now = millis();

  // Check grace period expiry
  if (gracePeriod_ && (now - graceStartTime_ >= STARTUP_GRACE_PERIOD_MS)) {
    gracePeriod_ = false;
    steering_.setStartupGracePeriod(false);
    Serial.println("Grace period expired - safety active");
  }

  // Command timeout check
  if (cmdReceived_ && (now - lastCmdTime_ > CMD_TIMEOUT_MS)) {
    if (!timeoutActive_) {
      timeoutActive_ = true;
      drive_.stop();
      steering_.setTargetAngle(0);
      Serial.println("SAFETY: Command timeout - motors stopped");
    }
  } else if (cmdReceived_) {
    timeoutActive_ = false;
  }

  // Emergency stop check and auto-recovery
  steering_.attemptAutoClearEmergencyStop();
}

void SafetyMonitor::commandReceived() {
  cmdReceived_ = true;
  lastCmdTime_ = millis();
}

bool SafetyMonitor::isEmergencyStop() const {
  return steering_.isEmergencyStop();
}

unsigned long SafetyMonitor::getCommandAge() const {
  if (!cmdReceived_)
    return UINT32_MAX;
  return millis() - lastCmdTime_;
}

void SafetyMonitor::setGracePeriod(bool enabled) {
  gracePeriod_ = enabled;
  if (enabled) {
    graceStartTime_ = millis();
    steering_.setStartupGracePeriod(true);
  }
}

void SafetyMonitor::tryResetEmergencyStop() {
  steering_.forceResetEmergencyStop();
  Serial.println("Emergency stop reset");
}
