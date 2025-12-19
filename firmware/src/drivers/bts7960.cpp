// ============================================================================
// BTS7960 Motor Driver Implementation
// ============================================================================

#include "drivers/bts7960.h"

BTS7960::BTS7960(uint8_t lpwmPin, uint8_t rpwmPin, uint8_t lpwmChannel,
                 uint8_t rpwmChannel)
    : lpwmPin_(lpwmPin), rpwmPin_(rpwmPin), lpwmChannel_(lpwmChannel),
      rpwmChannel_(rpwmChannel), currentSpeed_(0), enabled_(true) {}

void BTS7960::begin() {
  // Configure PWM channels
  ledcSetup(lpwmChannel_, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(rpwmChannel_, PWM_FREQUENCY, PWM_RESOLUTION);

  // Attach pins to channels
  ledcAttachPin(lpwmPin_, lpwmChannel_);
  ledcAttachPin(rpwmPin_, rpwmChannel_);

  // Start with motor stopped
  stop();
}

void BTS7960::setSpeed(int16_t speed) {
  if (!enabled_) {
    stop();
    return;
  }

  // Clamp to valid range
  currentSpeed_ = constrain(speed, -PWM_MAX_VALUE, PWM_MAX_VALUE);

  // Apply deadband to prevent motor hum at low speeds
  if (abs(currentSpeed_) < MOTOR_DEADBAND) {
    stop();
    return;
  }

  if (currentSpeed_ > 0) {
    // Forward direction
    ledcWrite(lpwmChannel_, currentSpeed_);
    ledcWrite(rpwmChannel_, 0);
  } else {
    // Reverse direction
    ledcWrite(lpwmChannel_, 0);
    ledcWrite(rpwmChannel_, abs(currentSpeed_));
  }
}

void BTS7960::stop() {
  ledcWrite(lpwmChannel_, 0);
  ledcWrite(rpwmChannel_, 0);
  currentSpeed_ = 0;
}

void BTS7960::setEnabled(bool enable) {
  enabled_ = enable;
  if (!enabled_) {
    stop();
  }
}
