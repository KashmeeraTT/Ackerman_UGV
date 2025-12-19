// ============================================================================
// Limit Switch Implementation
// ============================================================================

#include "hal/limit_switch.h"

LimitSwitch::LimitSwitch(uint8_t pin)
    : pin_(pin), triggered_(false), lastTriggerTime_(0), enabled_(true) {}

void LimitSwitch::begin() {
  // Configure pin with pullup - switch pulls to GND when pressed
  pinMode(pin_, INPUT_PULLUP);
}

bool LimitSwitch::isPressed() const {
  // Active LOW: pressed when pin reads LOW
  return digitalRead(pin_) == LOW;
}

void LimitSwitch::handleInterrupt() {
  if (!enabled_) {
    return;
  }

  unsigned long now = millis();

  // Debounce check
  if (now - lastTriggerTime_ >= LIMIT_DEBOUNCE_MS) {
    // Verify the switch is actually pressed (active LOW)
    if (digitalRead(pin_) == LOW) {
      triggered_ = true;
      lastTriggerTime_ = now;
    }
  }
}
