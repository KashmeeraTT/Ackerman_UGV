// ============================================================================
// Steering Controller
// ============================================================================
// PID-based steering control with encoder feedback and limit switches
// ============================================================================

#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include "config.h"
#include "drivers/bts7960.h"
#include "drivers/display.h"
#include "hal/encoder.h"
#include "hal/limit_switch.h"
#include <PID_v1.h>
#include <Preferences.h>

class SteeringController {
public:
  enum CalibrationState {
    NOT_CALIBRATED,
    CALIBRATING,
    CALIBRATED,
    CALIBRATION_ERROR
  };

  /**
   * @brief Construct steering controller
   * @param motor Reference to motor driver
   * @param encoder Reference to encoder
   * @param leftLimit Reference to left limit switch
   * @param rightLimit Reference to right limit switch
   */
  SteeringController(BTS7960 &motor, Encoder &encoder, LimitSwitch &leftLimit,
                     LimitSwitch &rightLimit);

  /**
   * @brief Initialize controller (call after motor/encoder/switch begin())
   */
  void begin();

  /**
   * @brief Set display for visual feedback during calibration
   */
  void setDisplay(Display *display) { display_ = display; }

  /**
   * @brief Main update loop - call at 100 Hz
   */
  void update();

  /**
   * @brief Set target steering angle
   * @param angleDeg Target angle in degrees (-10 to +10)
   */
  void setTargetAngle(float angleDeg);

  /**
   * @brief Get current steering angle
   */
  float getCurrentAngle() const { return currentAngle_; }

  /**
   * @brief Get target steering angle
   */
  float getTargetAngle() const { return targetAngle_; }

  /**
   * @brief Get encoder count
   */
  long getEncoderCount() const;

  /**
   * @brief Perform calibration routine
   * @return true if successful
   */
  bool calibrate();

  /**
   * @brief Get calibration state
   */
  CalibrationState getCalibrationState() const { return calibState_; }

  /**
   * @brief Check if emergency stop is active
   */
  bool isEmergencyStop() const { return emergencyStop_; }

  /**
   * @brief Clear emergency stop if in safe zone
   */
  void clearEmergencyStop();

  /**
   * @brief Force reset emergency stop
   */
  void forceResetEmergencyStop();

  /**
   * @brief Set startup grace period (suppresses limit switch triggers)
   */
  void setStartupGracePeriod(bool enabled);

  /**
   * @brief Attempt auto-clear of emergency stop if in safe zone
   */
  void attemptAutoClearEmergencyStop();

  /**
   * @brief Get left limit switch state
   */
  bool getLeftLimitState() const;

  /**
   * @brief Get right limit switch state
   */
  bool getRightLimitState() const;

private:
  BTS7960 &motor_;
  Encoder &encoder_;
  LimitSwitch &leftLimit_;
  LimitSwitch &rightLimit_;

  double targetAngle_;
  double smoothedTarget_;
  double currentAngle_;
  double filteredAngle_;
  double pidOutput_;

  CalibrationState calibState_;
  bool emergencyStop_;
  bool gracePeriod_;
  unsigned long lastUpdateTime_;

  float pulsesPerDegree_; // Dynamically calculated during calibration
  long dynamicSoftLimit_; // Calculated as pulsesPerDegree_ * 8 (for ±8 degrees)

  PID pid_;
  Preferences prefs_;
  Display *display_;

  void updateCurrentAngle();
  bool isInSafeZone() const;
  void saveCalibration();
  void loadCalibration();
};

#endif // STEERING_CONTROLLER_H
