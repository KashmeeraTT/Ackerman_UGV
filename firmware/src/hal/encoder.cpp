// ============================================================================
// Quadrature Encoder Implementation
// ============================================================================

#include "hal/encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
    : pinA_(pinA), pinB_(pinB), count_(0), lastA_(false), lastB_(false) {}

void Encoder::begin() {
  // Configure encoder pins as inputs
  // Note: GPIO 34-39 are input-only and don't support internal pullup
  pinMode(pinA_, INPUT);
  pinMode(pinB_, INPUT);

  // Read initial state
  lastA_ = digitalRead(pinA_);
  lastB_ = digitalRead(pinB_);
}

void Encoder::handleInterrupt() {
  // Read current states
  bool a = digitalRead(pinA_);
  bool b = digitalRead(pinB_);

  // Full 4x quadrature decoding
  // Count direction based on which channel changed and the state of the other
  if (a != lastA_) {
    // Channel A changed
    if (a == b) {
      count_++;
    } else {
      count_--;
    }
  }
  if (b != lastB_) {
    // Channel B changed
    if (a != b) {
      count_++;
    } else {
      count_--;
    }
  }

  lastA_ = a;
  lastB_ = b;
}
