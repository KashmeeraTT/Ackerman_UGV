// ============================================================================
// BTS7960 Motor Driver
// ============================================================================
// Controls a BTS7960 H-Bridge motor driver using two PWM pins
// ============================================================================

#ifndef BTS7960_H
#define BTS7960_H

#include "config.h"

class BTS7960 {
public:
  /**
   * @brief Construct a BTS7960 motor driver
   * @param lpwmPin GPIO pin for left (forward) PWM
   * @param rpwmPin GPIO pin for right (reverse) PWM
   * @param lpwmChannel PWM channel for LPWM
   * @param rpwmChannel PWM channel for RPWM
   */
  BTS7960(uint8_t lpwmPin, uint8_t rpwmPin, uint8_t lpwmChannel,
          uint8_t rpwmChannel);

  /**
   * @brief Initialize the motor driver (setup PWM channels)
   */
  void begin();

  /**
   * @brief Set motor speed and direction
   * @param speed Speed value (-255 to +255). Positive = forward, negative =
   * reverse
   */
  void setSpeed(int16_t speed);

  /**
   * @brief Stop the motor immediately
   */
  void stop();

  /**
   * @brief Enable or disable the motor
   * @param enable true to enable, false to disable
   */
  void setEnabled(bool enable);

  /**
   * @brief Get current speed setting
   * @return Current speed (-255 to +255)
   */
  int16_t getSpeed() const { return currentSpeed_; }

  /**
   * @brief Check if motor is enabled
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

#endif // BTS7960_H
