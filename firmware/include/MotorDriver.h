#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "pin_config.h"
#include <Arduino.h>

/**
 * @brief Motor driver class for BTS7960 H-Bridge
 *
 * Controls motor speed and direction using LPWM and RPWM pins.
 * BTS7960 Enable pins are assumed to be connected to HIGH (always enabled).
 */
class MotorDriver {
public:
  /**
   * @brief Construct a new Motor Driver object
   *
   * @param lpwmPin GPIO pin for left PWM (forward)
   * @param rpwmPin GPIO pin for right PWM (reverse)
   * @param lpwmChannel PWM channel for LPWM
   * @param rpwmChannel PWM channel for RPWM
   */
  MotorDriver(uint8_t lpwmPin, uint8_t rpwmPin, uint8_t lpwmChannel,
              uint8_t rpwmChannel)
      : lpwmPin_(lpwmPin), rpwmPin_(rpwmPin), lpwmChannel_(lpwmChannel),
        rpwmChannel_(rpwmChannel), currentSpeed_(0), enabled_(true) {}

  /**
   * @brief Initialize the motor driver
   * Sets up PWM channels and pins
   */
  void begin() {
    // Configure PWM channels
    ledcSetup(lpwmChannel_, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(rpwmChannel_, PWM_FREQUENCY, PWM_RESOLUTION);

    // Attach pins to channels
    ledcAttachPin(lpwmPin_, lpwmChannel_);
    ledcAttachPin(rpwmPin_, rpwmChannel_);

    // Start with motor stopped
    stop();
  }

  /**
   * @brief Set motor speed and direction
   *
   * @param speed Speed value (-255 to 255)
   *              Positive = forward, Negative = reverse, 0 = stop
   */
  void setSpeed(int16_t speed) {
    if (!enabled_) {
      stop();
      return;
    }

    currentSpeed_ = constrain(speed, -MAX_PWM_VALUE, MAX_PWM_VALUE);

    // Apply deadband to prevent motor hum at low speeds
    if (abs(currentSpeed_) < MOTOR_DEADBAND) {
      stop();
      return;
    }

    if (currentSpeed_ > 0) {
      // Forward direction
      ledcWrite(lpwmChannel_, currentSpeed_);
      ledcWrite(rpwmChannel_, 0);
    } else if (currentSpeed_ < 0) {
      // Reverse direction
      ledcWrite(lpwmChannel_, 0);
      ledcWrite(rpwmChannel_, abs(currentSpeed_));
    } else {
      stop();
    }
  }

  /**
   * @brief Stop the motor immediately
   */
  void stop() {
    ledcWrite(lpwmChannel_, 0);
    ledcWrite(rpwmChannel_, 0);
    currentSpeed_ = 0;
  }

  /**
   * @brief Enable or disable the motor
   *
   * @param enable true to enable, false to disable
   */
  void setEnabled(bool enable) {
    enabled_ = enable;
    if (!enabled_) {
      stop();
    }
  }

  /**
   * @brief Get current speed setting
   *
   * @return int16_t Current speed (-255 to 255)
   */
  int16_t getCurrentSpeed() const { return currentSpeed_; }

  /**
   * @brief Check if motor is enabled
   *
   * @return true if enabled
   */
  bool isEnabled() const { return enabled_; }

private:
  uint8_t lpwmPin_;
  uint8_t rpwmPin_;
  uint8_t lpwmChannel_;
  uint8_t rpwmChannel_;
  int16_t currentSpeed_;
  bool enabled_;
};

#endif // MOTOR_DRIVER_H
