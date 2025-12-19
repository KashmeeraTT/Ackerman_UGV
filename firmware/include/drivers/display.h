// ============================================================================
// OLED Display Driver
// ============================================================================
// SSD1306 OLED display for status information
// ============================================================================

#ifndef DISPLAY_H
#define DISPLAY_H

#include "config.h"
#include <Adafruit_SSD1306.h>

class Display {
public:
  Display();

  /**
   * @brief Initialize the display
   * @return true if display found
   */
  bool begin();

  /**
   * @brief Check if display is available
   */
  bool isAvailable() const { return available_; }

  /**
   * @brief Clear the display
   */
  void clear();

  /**
   * @brief Update the display (call after drawing)
   */
  void show();

  /**
   * @brief Show boot message
   */
  void showBoot(const char *message);

  /**
   * @brief Show calibration status
   * @param message Status message
   * @param leftLimit Left limit switch state
   * @param rightLimit Right limit switch state
   * @param encoderCount Current encoder count
   */
  void showCalibration(const char *message, bool leftLimit, bool rightLimit,
                       long encoderCount);

  /**
   * @brief Show main status screen
   * @param mode Mode string (READY, RUNNING, TIMEOUT, E-STOP)
   * @param rosConnected ROS connection status
   * @param cmdAgeMs Time since last command (ms)
   * @param currentAngle Current steering angle
   * @param targetAngle Target steering angle
   * @param velocity Current velocity
   * @param encoderCount Encoder count
   * @param leftLimit Left limit state
   * @param rightLimit Right limit state
   * @param statusCode Status code string
   */
  void showStatus(const char *mode, bool rosConnected, unsigned long cmdAgeMs,
                  float currentAngle, float targetAngle, float velocity,
                  long encoderCount, bool leftLimit, bool rightLimit,
                  const char *statusCode);

  /**
   * @brief Show calibration progress with step indicator and timeout
   * @param step Current step (1-4): 1=LEFT, 2=RIGHT, 3=CENTER, 4=DONE
   * @param stepName Name of current step
   * @param progress Progress percentage (0-100)
   * @param timeoutSec Remaining timeout seconds
   * @param encoderCount Current encoder count
   */
  void showCalibrationProgress(int step, const char *stepName, int progress,
                               int timeoutSec, long encoderCount);

  /**
   * @brief Show calibration success with results
   * @param totalPulses Total encoder pulses measured
   * @param pulsesPerDeg Pulses per degree calculated
   */
  void showCalibrationSuccess(long totalPulses, float pulsesPerDeg);

  /**
   * @brief Show calibration failure with reason
   * @param reason Failure reason string
   * @param step Step where failure occurred
   */
  void showCalibrationFailed(const char *reason, int step);

  /**
   * @brief Show error message
   * @param error Error string
   */
  void showError(const char *error);

private:
  Adafruit_SSD1306 oled_;
  bool available_;
};

#endif // DISPLAY_H
