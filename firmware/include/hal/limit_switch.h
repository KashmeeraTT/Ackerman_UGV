// ============================================================================
// Limit Switch Handler
// ============================================================================
// Handles limit switches with debouncing and interrupt support
// ============================================================================

#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include "config.h"

class LimitSwitch {
public:
  /**
   * @brief Construct a limit switch handler
   * @param pin GPIO pin (will use INPUT_PULLUP, active LOW)
   */
  explicit LimitSwitch(uint8_t pin);

  /**
   * @brief Initialize the limit switch pin
   */
  void begin();

  /**
   * @brief Check if switch is currently pressed (with debouncing)
   * @return true if pressed (LOW state)
   */
  bool isPressed() const;

  /**
   * @brief Check if switch was triggered (latched)
   * @return true if switch was hit since last clear
   */
  bool wasTriggered() const { return triggered_; }

  /**
   * @brief Clear the triggered flag
   */
  void clearTriggered() { triggered_ = false; }

  /**
   * @brief Handle interrupt - call from ISR
   * Sets triggered flag with debouncing
   */
  void handleInterrupt();

  /**
   * @brief Enable or disable interrupt handling
   * @param enabled true to enable ISR, false to ignore
   */
  void setEnabled(bool enabled) { enabled_ = enabled; }

  /**
   * @brief Get the GPIO pin
   */
  uint8_t getPin() const { return pin_; }

private:
  uint8_t pin_;
  volatile bool triggered_;
  volatile unsigned long lastTriggerTime_;
  bool enabled_;
};

#endif // LIMIT_SWITCH_H
