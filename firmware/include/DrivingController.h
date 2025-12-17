#ifndef DRIVING_CONTROLLER_H
#define DRIVING_CONTROLLER_H

#include "MotorDriver.h"
#include "pin_config.h"
#include <Arduino.h>

/**
 * @brief Driving motor controller with velocity control
 *
 * Manages the driving motor with:
 * - Velocity command input (-1.0 to 1.0 m/s)
 * - Smooth acceleration/deceleration
 * - Open-loop control (no encoder feedback)
 */
class DrivingController {
public:
  DrivingController(MotorDriver &motor)
      : motor_(motor), targetVelocity_(0.0), currentVelocity_(0.0),
        maxAcceleration_(0.5), // 0.5 m/s per update (at 100Hz = 5 m/s²)
        maxVelocity_(1.0) {}   // Maximum velocity in m/s

  /**
   * @brief Initialize driving controller
   */
  void begin() {
    motor_.begin();
    motor_.stop();
  }

  /**
   * @brief Main update loop - call this regularly (10ms recommended)
   * Implements smooth acceleration to target velocity
   */
  void update() {
    // Smooth acceleration
    float velocityError = targetVelocity_ - currentVelocity_;

    if (abs(velocityError) < maxAcceleration_) {
      currentVelocity_ = targetVelocity_;
    } else if (velocityError > 0) {
      currentVelocity_ += maxAcceleration_;
    } else {
      currentVelocity_ -= maxAcceleration_;
    }

    // Convert velocity to PWM
    int16_t pwmValue = velocityToPWM(currentVelocity_);

    // Apply to motor
    motor_.setSpeed(pwmValue);
  }

  /**
   * @brief Set target velocity
   *
   * @param velocity Target velocity in m/s (-maxVelocity to +maxVelocity)
   *                 Positive = forward, Negative = reverse
   */
  void setVelocity(float velocity) {
    targetVelocity_ = constrain(velocity, -maxVelocity_, maxVelocity_);
  }

  /**
   * @brief Get current velocity
   *
   * @return float Current velocity in m/s
   */
  float getCurrentVelocity() const { return currentVelocity_; }

  /**
   * @brief Get target velocity
   *
   * @return float Target velocity in m/s
   */
  float getTargetVelocity() const { return targetVelocity_; }

  /**
   * @brief Stop the motor immediately
   */
  void stop() {
    targetVelocity_ = 0.0;
    currentVelocity_ = 0.0;
    motor_.stop();
  }

  /**
   * @brief Set maximum velocity
   *
   * @param maxVel Maximum velocity in m/s
   */
  void setMaxVelocity(float maxVel) { maxVelocity_ = abs(maxVel); }

  /**
   * @brief Set maximum acceleration
   *
   * @param maxAccel Maximum acceleration (m/s per update cycle)
   */
  void setMaxAcceleration(float maxAccel) { maxAcceleration_ = abs(maxAccel); }

private:
  MotorDriver &motor_;
  float targetVelocity_;
  float currentVelocity_;
  float maxAcceleration_;
  float maxVelocity_;

  /**
   * @brief Convert velocity (m/s) to PWM value
   *
   * @param velocity Velocity in m/s
   * @return int16_t PWM value (-255 to 255)
   *
   * Note: This is a simple linear mapping. You may need to calibrate
   * this based on your actual motor characteristics and wheel diameter.
   */
  int16_t velocityToPWM(float velocity) {
    // Simple linear mapping
    // Adjust the scale factor based on your motor/wheel specs
    // For now: 1.0 m/s = 255 PWM
    
    // Add a small offset to account for motor deadband
    // When PWM < MOTOR_DEADBAND, motor stops
    // So we need to map velocity more aggressively
    
    if (abs(velocity) < 0.05) {
      // For very small velocities, just stop
      return 0;
    }
    
    // Scale velocity to PWM, accounting for deadband
    // Map: 0.05 m/s -> MOTOR_DEADBAND PWM, 1.0 m/s -> 255 PWM
    float pwmFloat = (velocity / maxVelocity_) * MAX_PWM_VALUE;
    
    // Ensure we meet the deadband requirement
    if (pwmFloat > 0 && pwmFloat < MOTOR_DEADBAND) {
      pwmFloat = MOTOR_DEADBAND;
    } else if (pwmFloat < 0 && pwmFloat > -MOTOR_DEADBAND) {
      pwmFloat = -MOTOR_DEADBAND;
    }
    
    return (int16_t)constrain(pwmFloat, -MAX_PWM_VALUE, MAX_PWM_VALUE);
  }
};

#endif // DRIVING_CONTROLLER_H
