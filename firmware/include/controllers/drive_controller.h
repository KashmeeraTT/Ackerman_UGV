// ============================================================================
// Drive Controller
// ============================================================================
// Open-loop velocity control for driving motor
// ============================================================================

#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H

#include "config.h"
#include "drivers/bts7960.h"

class DriveController {
public:
  /**
   * @brief Construct drive controller
   * @param motor Reference to motor driver
   */
  explicit DriveController(BTS7960 &motor);

  /**
   * @brief Initialize controller
   */
  void begin();

  /**
   * @brief Main update loop - call at 100 Hz
   * Implements smooth acceleration
   */
  void update();

  /**
   * @brief Set target velocity
   * @param velocity Target velocity in m/s (-MAX to +MAX)
   */
  void setVelocity(float velocity);

  /**
   * @brief Get current velocity
   */
  float getCurrentVelocity() const { return currentVelocity_; }

  /**
   * @brief Get target velocity
   */
  float getTargetVelocity() const { return targetVelocity_; }

  /**
   * @brief Stop immediately
   */
  void stop();

  /**
   * @brief Set maximum velocity
   */
  void setMaxVelocity(float maxVel) { maxVelocity_ = abs(maxVel); }

  /**
   * @brief Set maximum acceleration (m/s per update)
   */
  void setMaxAcceleration(float maxAccel) { maxAccel_ = abs(maxAccel); }

private:
  BTS7960 &motor_;
  float targetVelocity_;
  float currentVelocity_;
  float maxAccel_;
  float maxVelocity_;

  int16_t velocityToPWM(float velocity);
};

#endif // DRIVE_CONTROLLER_H
