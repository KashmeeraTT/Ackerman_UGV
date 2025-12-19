// ============================================================================
// Quadrature Encoder
// ============================================================================
// Reads a quadrature encoder using interrupts
// ============================================================================

#ifndef ENCODER_H
#define ENCODER_H

#include "config.h"

class Encoder {
public:
  /**
   * @brief Construct an encoder handler
   * @param pinA GPIO pin for channel A
   * @param pinB GPIO pin for channel B
   */
  Encoder(uint8_t pinA, uint8_t pinB);

  /**
   * @brief Initialize encoder pins
   * Note: Interrupt attachment must be done in main setup() using getISR()
   */
  void begin();

  /**
   * @brief Get current encoder count
   * @return Encoder count (can be negative)
   */
  long getCount() const { return count_; }

  /**
   * @brief Reset encoder count to zero
   */
  void reset() { count_ = 0; }

  /**
   * @brief Set encoder count to a specific value
   * @param value New count value
   */
  void setCount(long value) { count_ = value; }

  /**
   * @brief Get angle in degrees from encoder count
   * @param pulsesPerDegree Calibration value
   * @return Angle in degrees
   */
  float getAngle(float pulsesPerDegree) const {
    return static_cast<float>(count_) / pulsesPerDegree;
  }

  /**
   * @brief Handle encoder interrupt - call from ISR
   * This is public so it can be called from a static ISR wrapper
   */
  void handleInterrupt();

  // Pins for external access (needed for interrupt attachment)
  uint8_t getPinA() const { return pinA_; }
  uint8_t getPinB() const { return pinB_; }

private:
  uint8_t pinA_;
  uint8_t pinB_;
  volatile long count_;
  volatile bool lastA_;
  volatile bool lastB_;
};

#endif // ENCODER_H
