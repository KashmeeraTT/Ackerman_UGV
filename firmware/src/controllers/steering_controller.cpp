// ============================================================================
// Steering Controller Implementation
// ============================================================================

#include "controllers/steering_controller.h"

SteeringController::SteeringController(BTS7960 &motor, Encoder &encoder,
                                       LimitSwitch &leftLimit,
                                       LimitSwitch &rightLimit)
    : motor_(motor), encoder_(encoder), leftLimit_(leftLimit),
      rightLimit_(rightLimit), targetAngle_(0), smoothedTarget_(0),
      currentAngle_(0), filteredAngle_(0), pidOutput_(0),
      calibState_(NOT_CALIBRATED), emergencyStop_(false), gracePeriod_(false),
      lastUpdateTime_(0), pulsesPerDegree_(PULSES_PER_DEGREE),
      dynamicSoftLimit_(SOFT_LIMIT_PULSES),
      pid_(&currentAngle_, &pidOutput_, &smoothedTarget_, STEERING_KP,
           STEERING_KI, STEERING_KD, DIRECT),
      display_(nullptr) {}

void SteeringController::begin() {
  // Configure PID
  pid_.SetMode(AUTOMATIC);
  pid_.SetOutputLimits(-PWM_MAX_VALUE * 0.95, PWM_MAX_VALUE * 0.95);
  pid_.SetSampleTime(STEERING_PID_SAMPLE_MS);

  // Load calibration from NVS
  loadCalibration();
}

void SteeringController::update() {
  if (emergencyStop_) {
    motor_.stop();
    return;
  }

  if (calibState_ != CALIBRATED) {
    return;
  }

  // Calculate dt
  unsigned long now = millis();
  float dt = (now - lastUpdateTime_) / 1000.0f;
  if (dt <= 0)
    dt = 0.01f;
  lastUpdateTime_ = now;

  // Update current angle with filtering
  updateCurrentAngle();

  // Rate-limited target smoothing (max 30 deg/sec)
  const float MAX_RATE = 30.0f;
  float maxChange = MAX_RATE * dt;
  float delta = targetAngle_ - smoothedTarget_;
  delta = constrain(delta, -maxChange, maxChange);
  smoothedTarget_ += delta;

  // Clamp to limits
  smoothedTarget_ = constrain(smoothedTarget_, MIN_STEERING_ANGLE_DEG,
                              MAX_STEERING_ANGLE_DEG);

  // Extra safety at limits
  if ((currentAngle_ >= MAX_STEERING_ANGLE_DEG - 0.5 &&
       smoothedTarget_ > currentAngle_) ||
      (currentAngle_ <= MIN_STEERING_ANGLE_DEG + 0.5 &&
       smoothedTarget_ < currentAngle_)) {
    smoothedTarget_ = currentAngle_;
  }

  // Check soft limits using dynamically calculated value
  long count = encoder_.getCount();
  if (count < -dynamicSoftLimit_ || count > dynamicSoftLimit_) {
    emergencyStop_ = true;
    motor_.stop();
    Serial.print("SOFT LIMIT HIT! Encoder: ");
    Serial.print(count);
    Serial.print(" Limit: +/- ");
    Serial.println(dynamicSoftLimit_);
    return;
  }

  // Compute PID
  pid_.Compute();

  // Apply motor command with inversion
  motor_.setSpeed(static_cast<int16_t>(pidOutput_ * STEERING_MOTOR_INVERT));
}

void SteeringController::setTargetAngle(float angleDeg) {
  // Constrain to soft limits (±8°) - keep 2° overhead before physical limits
  // (±10°)
  targetAngle_ =
      constrain(angleDeg, -SOFT_LIMIT_ANGLE_DEG, SOFT_LIMIT_ANGLE_DEG);
}

long SteeringController::getEncoderCount() const { return encoder_.getCount(); }

bool SteeringController::calibrate() {
  calibState_ = CALIBRATING;
  emergencyStop_ = false;

  // IMPORTANT: Enable limit switches for calibration
  // They may have been disabled by grace period
  leftLimit_.setEnabled(true);
  rightLimit_.setEnabled(true);
  leftLimit_.clearTriggered();
  rightLimit_.clearTriggered();

  const int STEP_LEFT = 1;
  const int STEP_RIGHT = 2;
  const int STEP_CENTER = 3;
  const int STEP_DONE = 4;
  const unsigned long UPDATE_INTERVAL_MS = 100;

  // Pre-calibration check
  Serial.println("\n========================================");
  Serial.println("CALIBRATION STARTING");
  Serial.println("========================================");

  // Check if limit switches are already pressed
  if (leftLimit_.isPressed()) {
    Serial.println("ERROR: Left limit switch already pressed!");
    if (display_)
      display_->showCalibrationFailed("Left switch stuck", 0);
    calibState_ = CALIBRATION_ERROR;
    return false;
  }
  if (rightLimit_.isPressed()) {
    Serial.println("ERROR: Right limit switch already pressed!");
    if (display_)
      display_->showCalibrationFailed("Right switch stuck", 0);
    calibState_ = CALIBRATION_ERROR;
    return false;
  }

  // ========================================
  // STEP 1: Move to LEFT limit
  // ========================================
  Serial.println("\n[Step 1/4] Moving to LEFT limit...");
  unsigned long startTime = millis();
  unsigned long lastUpdate = 0;

  motor_.setSpeed(-CALIBRATION_SPEED * STEERING_MOTOR_INVERT);

  while (!leftLimit_.wasTriggered()) {
    unsigned long elapsed = millis() - startTime;
    int remainingSec = (CALIBRATION_TIMEOUT_MS - elapsed) / 1000;

    if (elapsed >= CALIBRATION_TIMEOUT_MS) {
      motor_.stop();
      Serial.println("ERROR: LEFT limit timeout!");
      if (display_)
        display_->showCalibrationFailed("Timeout LEFT", STEP_LEFT);
      calibState_ = CALIBRATION_ERROR;
      return false;
    }

    // Update display every UPDATE_INTERVAL_MS
    if (millis() - lastUpdate >= UPDATE_INTERVAL_MS) {
      lastUpdate = millis();
      int progress = (elapsed * 100) / CALIBRATION_TIMEOUT_MS;
      if (display_) {
        display_->showCalibrationProgress(STEP_LEFT, "<< LEFT", progress,
                                          remainingSec, encoder_.getCount());
      }
    }
    delay(10);
  }
  motor_.stop();

  long leftCount = encoder_.getCount();
  Serial.print("  LEFT OK! Encoder: ");
  Serial.println(leftCount);
  delay(300);

  // ========================================
  // STEP 2: Move to RIGHT limit
  // ========================================
  Serial.println("\n[Step 2/4] Moving to RIGHT limit...");
  leftLimit_.clearTriggered();
  rightLimit_.clearTriggered();
  startTime = millis();

  motor_.setSpeed(CALIBRATION_SPEED * STEERING_MOTOR_INVERT);

  while (!rightLimit_.wasTriggered()) {
    unsigned long elapsed = millis() - startTime;
    int remainingSec = (CALIBRATION_TIMEOUT_MS - elapsed) / 1000;

    if (elapsed >= CALIBRATION_TIMEOUT_MS) {
      motor_.stop();
      Serial.println("ERROR: RIGHT limit timeout!");
      if (display_)
        display_->showCalibrationFailed("Timeout RIGHT", STEP_RIGHT);
      calibState_ = CALIBRATION_ERROR;
      return false;
    }

    if (millis() - lastUpdate >= UPDATE_INTERVAL_MS) {
      lastUpdate = millis();
      int progress = (elapsed * 100) / CALIBRATION_TIMEOUT_MS;
      if (display_) {
        display_->showCalibrationProgress(STEP_RIGHT, "RIGHT >>", progress,
                                          remainingSec, encoder_.getCount());
      }
    }
    delay(10);
  }
  motor_.stop();

  long rightCount = encoder_.getCount();
  Serial.print("  RIGHT OK! Encoder: ");
  Serial.println(rightCount);
  delay(300);

  // ========================================
  // Calculate calibration values
  // ========================================
  long center = (leftCount + rightCount) / 2;
  long totalRange = abs(rightCount - leftCount);
  float pulsesPerDeg = totalRange / 20.0f; // 20 degrees total travel

  Serial.println("\n========================================");
  Serial.println("CALIBRATION RESULTS");
  Serial.println("========================================");
  Serial.print("Range: ");
  Serial.print(totalRange);
  Serial.println(" pulses (20 degrees)");
  Serial.print("Pulses/10deg: ");
  Serial.println(totalRange / 2);
  Serial.print("Pulses/degree: ");
  Serial.println(pulsesPerDeg, 1);
  Serial.print("Center position: ");
  Serial.println(center);

  // Sanity check - but allow low values for testing without encoder
  if (totalRange < 10) {
    Serial.println("ERROR: No encoder movement detected!");
    Serial.println("Check: Is encoder connected to GPIO 34 & 35?");
    if (display_)
      display_->showCalibrationFailed("No encoder data", STEP_RIGHT);
    calibState_ = CALIBRATION_ERROR;
    return false;
  }

  if (totalRange > 50000) {
    Serial.println("ERROR: Encoder range too large - possible noise!");
    if (display_)
      display_->showCalibrationFailed("Encoder noise", STEP_RIGHT);
    calibState_ = CALIBRATION_ERROR;
    return false;
  }

  // Warn if range seems low but continue
  if (totalRange < 200) {
    Serial.println(
        "WARNING: Low encoder range - calibration may be inaccurate");
  }

  // Store dynamically calculated values for use during normal operation
  pulsesPerDegree_ = pulsesPerDeg;
  dynamicSoftLimit_ = static_cast<long>(pulsesPerDeg * SOFT_LIMIT_ANGLE_DEG);

  Serial.print("Dynamic soft limit: +/- ");
  Serial.print(dynamicSoftLimit_);
  Serial.print(" pulses (");
  Serial.print(SOFT_LIMIT_ANGLE_DEG, 0);
  Serial.println(" degrees)");

  // Show success screen
  if (display_) {
    display_->showCalibrationSuccess(totalRange, pulsesPerDeg);
  }

  // ========================================
  // STEP 3: Move to CENTER
  // ========================================
  Serial.println("\n[Step 3/4] Moving to center...");
  Serial.print("Current encoder: ");
  Serial.println(encoder_.getCount());
  Serial.print("Target center: ");
  Serial.println(center);

  rightLimit_.clearTriggered();
  leftLimit_.clearTriggered();
  startTime = millis();

  // Determine direction to reach center from current position
  // During calibration: Step 1 (LEFT) increased encoder, Step 2 (RIGHT)
  // decreased encoder Step 1 used: -CALIBRATION_SPEED * STEERING_MOTOR_INVERT
  // (LEFT) Step 2 used: +CALIBRATION_SPEED * STEERING_MOTOR_INVERT  (RIGHT)
  int centerSpeed = 0;
  long currentPos = encoder_.getCount();

  if (currentPos < center) {
    // We're below center (e.g., at -139, center is 6232)
    // Need to INCREASE encoder by moving LEFT (like Step 1)
    centerSpeed = -CALIBRATION_SPEED / 2 * STEERING_MOTOR_INVERT;
    Serial.println("Moving LEFT toward center (increasing encoder)...");
  } else {
    // We're above center
    // Need to DECREASE encoder by moving RIGHT (like Step 2)
    centerSpeed = CALIBRATION_SPEED / 2 * STEERING_MOTOR_INVERT;
    Serial.println("Moving RIGHT toward center (decreasing encoder)...");
  }

  motor_.setSpeed(centerSpeed);

  // Loop until we're within 50 pulses of center
  while (abs(encoder_.getCount() - center) > 50) {
    unsigned long elapsed = millis() - startTime;

    if (elapsed >= CALIBRATION_TIMEOUT_MS) {
      motor_.stop();
      Serial.println("ERROR: CENTER timeout!");
      Serial.print("Current encoder: ");
      Serial.println(encoder_.getCount());
      if (display_)
        display_->showCalibrationFailed("Timeout CENTER", STEP_CENTER);
      calibState_ = CALIBRATION_ERROR;
      return false;
    }

    if (millis() - lastUpdate >= UPDATE_INTERVAL_MS) {
      lastUpdate = millis();
      long remaining = abs(encoder_.getCount() - center);
      long total = abs(rightCount - center);
      int progress = (total > 0) ? (100 - (remaining * 100 / total)) : 100;
      progress = constrain(progress, 0, 100);
      int remainingSec = (CALIBRATION_TIMEOUT_MS - elapsed) / 1000;
      if (display_) {
        display_->showCalibrationProgress(STEP_CENTER, "CENTER", progress,
                                          remainingSec, encoder_.getCount());
      }
    }
    delay(10);
  }
  motor_.stop();

  Serial.print("Centering complete! Final encoder: ");
  Serial.println(encoder_.getCount());

  // Reset encoder at center
  encoder_.reset();
  saveCalibration();

  // ========================================
  // STEP 4: Complete
  // ========================================
  Serial.println("\n[Step 4/4] Calibration complete!");
  Serial.println("========================================\n");

  if (display_) {
    display_->showCalibrationProgress(STEP_DONE, "DONE!", 100, 0, 0);
  }
  delay(1000);

  calibState_ = CALIBRATED;
  emergencyStop_ = false;
  return true;
}

void SteeringController::clearEmergencyStop() {
  if (!leftLimit_.wasTriggered() && !rightLimit_.wasTriggered()) {
    emergencyStop_ = false;
  }
}

void SteeringController::forceResetEmergencyStop() {
  emergencyStop_ = false;
  leftLimit_.clearTriggered();
  rightLimit_.clearTriggered();
}

void SteeringController::setStartupGracePeriod(bool enabled) {
  gracePeriod_ = enabled;
  leftLimit_.setEnabled(!enabled);
  rightLimit_.setEnabled(!enabled);
}

void SteeringController::attemptAutoClearEmergencyStop() {
  if (emergencyStop_ && !gracePeriod_ && isInSafeZone()) {
    leftLimit_.clearTriggered();
    rightLimit_.clearTriggered();
    emergencyStop_ = false;
    Serial.println("Emergency stop auto-cleared");
  }
}

bool SteeringController::getLeftLimitState() const {
  return leftLimit_.isPressed();
}

bool SteeringController::getRightLimitState() const {
  return rightLimit_.isPressed();
}

void SteeringController::updateCurrentAngle() {
  float rawAngle = encoder_.getAngle(PULSES_PER_DEGREE);

  // EMA filter (alpha = 0.3)
  const float ALPHA = 0.3f;
  filteredAngle_ = ALPHA * rawAngle + (1.0f - ALPHA) * filteredAngle_;
  currentAngle_ = filteredAngle_;
}

bool SteeringController::isInSafeZone() const {
  long count = encoder_.getCount();
  return (count >= -SOFT_LIMIT_PULSES && count <= SOFT_LIMIT_PULSES);
}

void SteeringController::saveCalibration() {
  prefs_.begin("steering", false);
  prefs_.putBool("calibrated", true);
  prefs_.end();
}

void SteeringController::loadCalibration() {
  prefs_.begin("steering", true);
  bool isCalibrated = prefs_.getBool("calibrated", false);
  if (isCalibrated) {
    calibState_ = CALIBRATED;
  }
  prefs_.end();
}
