// ============================================================================
// Safety Monitor
// ============================================================================
// Centralized safety monitoring for command timeout and emergency stop
// ============================================================================

#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include "config.h"
#include "controllers/drive_controller.h"
#include "controllers/steering_controller.h"

class SafetyMonitor {
public:
  /**
   * @brief Construct safety monitor
   * @param steering Reference to steering controller
   * @param drive Reference to drive controller
   */
  SafetyMonitor(SteeringController &steering, DriveController &drive);

  /**
   * @brief Update safety checks - call at 100 Hz
   */
  void update();

  /**
   * @brief Notify that a command was received
   */
  void commandReceived();

  /**
   * @brief Check if command timeout is active
   */
  bool isTimeout() const { return timeoutActive_; }

  /**
   * @brief Check if emergency stop is active
   */
  bool isEmergencyStop() const;

  /**
   * @brief Get time since last command (ms)
   */
  unsigned long getCommandAge() const;

  /**
   * @brief Set startup grace period
   * @param enabled true to enable grace period
   */
  void setGracePeriod(bool enabled);

  /**
   * @brief Check if system is in grace period
   */
  bool inGracePeriod() const { return gracePeriod_; }

  /**
   * @brief Try to reset emergency stop
   */
  void tryResetEmergencyStop();

private:
  SteeringController &steering_;
  DriveController &drive_;

  bool cmdReceived_;
  unsigned long lastCmdTime_;
  bool timeoutActive_;
  bool gracePeriod_;
  unsigned long graceStartTime_;
};

#endif // SAFETY_MONITOR_H
