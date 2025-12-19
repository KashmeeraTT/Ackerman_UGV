#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include "MotorDriver.h"
#include "pin_config.h"
#include <Arduino.h>
#include <PID_v1.h>
#include <Preferences.h>

// Motor direction inversion: set to -1 if motor goes wrong direction
#define STEERING_MOTOR_INVERT -1

/**
 * @brief Steering controller with encoder feedback and safety limits
 *
 * Manages steering motor with:
 * - Encoder-based position tracking
 * - PID control for angle setpoints
 * - Hard ±10° safety limits
 * - Limit switch handling
 * - Auto-calibration on startup
 */
class SteeringController {
public:
  enum CalibrationState {
    NOT_CALIBRATED,
    CALIBRATING,
    CALIBRATED,
    CALIBRATION_ERROR
  };

  SteeringController(MotorDriver &motor)
      : motor_(motor), encoderCount_(0), centerEncoderCount_(0),
        targetAngle_(0.0), smoothedTargetAngle_(0.0), currentAngle_(0.0),
        filteredAngle_(0.0), pidOutput_(0.0), calibrationState_(NOT_CALIBRATED),
        leftLimitHit_(false), rightLimitHit_(false), emergencyStop_(false),
        startupGracePeriodEnabled_(false), lastLeftLimitTime_(0),
        lastRightLimitTime_(0), lastUpdateTime_(0),
        pid_(&currentAngle_, &pidOutput_, &smoothedTargetAngle_, 2.5, 0.1, 0.5,
             DIRECT) {

    // PID configuration
    pid_.SetMode(AUTOMATIC);
    pid_.SetOutputLimits(-MAX_PWM_VALUE * 0.95,
                         MAX_PWM_VALUE *
                             0.95); // Leave headroom for anti-windup
    pid_.SetSampleTime(10);         // 10ms sample time
  }

  /**
   * @brief Initialize steering controller
   * Sets up encoder pins, limit switches, and loads calibration from memory
   */
  void begin() {
    // Initialize motor
    motor_.begin();

    // Setup encoder pins
    pinMode(ENCODER_A_PIN, INPUT);
    pinMode(ENCODER_B_PIN, INPUT);

    // Attach encoder interrupts
    attachInterrupt(
        digitalPinToInterrupt(ENCODER_A_PIN),
        []() { instance_->handleEncoderISR(); }, CHANGE);

    // Setup limit switch pins - connected to GND (active LOW)
    // Note: Switches are connected to GND, not 5V
    // When pressed, they pull the pin to GND (LOW)
    // Use INPUT_PULLUP to keep pin HIGH when switch is open
    // When switch closes, it pulls pin to GND, triggering FALLING edge
    pinMode(LIMIT_SWITCH_LEFT, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_RIGHT, INPUT_PULLUP);

    // Attach limit switch interrupts
#if LIMIT_SWITCH_ACTIVE_HIGH
    // NC switches: trigger on RISING edge (LOW→HIGH when pressed)
    attachInterrupt(
        digitalPinToInterrupt(LIMIT_SWITCH_LEFT),
        []() { instance_->handleLeftLimitISR(); }, RISING);
    attachInterrupt(
        digitalPinToInterrupt(LIMIT_SWITCH_RIGHT),
        []() { instance_->handleRightLimitISR(); }, RISING);
#else
    // NO switches: trigger on FALLING edge (HIGH→LOW when pressed)
    attachInterrupt(
        digitalPinToInterrupt(LIMIT_SWITCH_LEFT),
        []() { instance_->handleLeftLimitISR(); }, FALLING);
    attachInterrupt(
        digitalPinToInterrupt(LIMIT_SWITCH_RIGHT),
        []() { instance_->handleRightLimitISR(); }, FALLING);
#endif

    // Load calibration from memory
    loadCalibration();
  }

  /**
   * @brief Main update loop - call this regularly (10ms recommended)
   * Updates PID control and motor output with rate limiting and filtering
   */
  void update() {
    if (emergencyStop_) {
      motor_.stop();
      return;
    }

    if (calibrationState_ != CALIBRATED) {
      return; // Don't control until calibrated
    }

    // Calculate dt for rate limiting
    unsigned long now = millis();
    float dt = (now - lastUpdateTime_) / 1000.0f;
    if (dt <= 0)
      dt = 0.01f; // Prevent division by zero
    lastUpdateTime_ = now;

    // Update current angle from encoder with EMA filter
    updateCurrentAngle();

    // Apply steering rate limiting to smooth target changes
    // Max rate: 30 degrees per second
    const float MAX_STEER_RATE = 30.0f; // deg/sec
    float maxChange = MAX_STEER_RATE * dt;
    float delta = targetAngle_ - smoothedTargetAngle_;
    delta = constrain(delta, -maxChange, maxChange);
    smoothedTargetAngle_ += delta;

    // Clamp smoothed target to safe limits
    smoothedTargetAngle_ = constrain(
        smoothedTargetAngle_, MIN_STEERING_ANGLE_DEG, MAX_STEERING_ANGLE_DEG);

    // Additional safety: If at limit and trying to go further, stop
    if ((currentAngle_ >= MAX_STEERING_ANGLE_DEG - 0.5 &&
         smoothedTargetAngle_ > currentAngle_) ||
        (currentAngle_ <= MIN_STEERING_ANGLE_DEG + 0.5 &&
         smoothedTargetAngle_ < currentAngle_)) {
      smoothedTargetAngle_ = currentAngle_;
    }

    // Compute PID
    pid_.Compute();

    // Anti-windup: if output is saturated, prevent integral buildup
    if (abs(pidOutput_) >= MAX_PWM_VALUE * 0.9) {
      // Near saturation - PID library handles this via output limits
    }

    // Apply motor command
    motor_.setSpeed((int16_t)(pidOutput_ * STEERING_MOTOR_INVERT));
  }

  /**
   * @brief Set target steering angle
   *
   * @param angleDeg Target angle in degrees (-10 to +10)
   */
  void setTargetAngle(float angleDeg) {
    targetAngle_ =
        constrain(angleDeg, MIN_STEERING_ANGLE_DEG, MAX_STEERING_ANGLE_DEG);
  }

  /**
   * @brief Get current steering angle
   *
   * @return float Current angle in degrees
   */
  float getCurrentAngle() const { return currentAngle_; }

  /**
   * @brief Get target steering angle
   *
   * @return float Target angle in degrees
   */
  float getTargetAngle() const { return targetAngle_; }

  /**
   * @brief Perform calibration routine with timeout protection
   * Moves to both limits and finds center
   * @return true if calibration successful, false if timeout
   */
  bool calibrate() {
    // Uses CALIBRATION_TIMEOUT_MS from pin_config.h

    calibrationState_ = CALIBRATING;
    emergencyStop_ = false;
    leftLimitHit_ = false;
    rightLimitHit_ = false;

    Serial.println("Calibration: Moving to LEFT limit...");
    unsigned long startTime = millis();

    // Step 1: Move left until limit switch (with timeout)
    motor_.setSpeed(-CALIBRATION_SPEED * STEERING_MOTOR_INVERT);
    while (!leftLimitHit_ && (millis() - startTime < CALIBRATION_TIMEOUT_MS)) {
      delay(10);
    }
    motor_.stop();

    if (!leftLimitHit_) {
      Serial.println("ERROR: LEFT limit timeout! Check wiring.");
      calibrationState_ = CALIBRATION_ERROR;
      return false;
    }

    long leftLimitCount = encoderCount_;
    Serial.print("  LEFT OK. Encoder: ");
    Serial.println(leftLimitCount);
    delay(300);

    // Step 2: Move right until limit switch (with timeout)
    leftLimitHit_ = false;
    rightLimitHit_ = false;
    startTime = millis();

    Serial.println("Calibration: Moving to RIGHT limit...");
    motor_.setSpeed(CALIBRATION_SPEED * STEERING_MOTOR_INVERT);
    while (!rightLimitHit_ && (millis() - startTime < CALIBRATION_TIMEOUT_MS)) {
      delay(10);
    }
    motor_.stop();

    if (!rightLimitHit_) {
      Serial.println("ERROR: RIGHT limit timeout! Check wiring.");
      calibrationState_ = CALIBRATION_ERROR;
      return false;
    }

    long rightLimitCount = encoderCount_;
    Serial.print("  RIGHT OK. Encoder: ");
    Serial.println(rightLimitCount);
    delay(300);

    // Step 3: Calculate center and show results
    centerEncoderCount_ = (leftLimitCount + rightLimitCount) / 2;
    long totalRange = rightLimitCount - leftLimitCount;

    Serial.println("\n=== CALIBRATION RESULTS ===");
    Serial.print("Range: ");
    Serial.print(totalRange);
    Serial.println(" pulses (20deg)");
    Serial.print("Pulses/10deg: ");
    Serial.println(totalRange / 2);
    Serial.println("===========================\n");

    // Step 4: Move to center (with timeout)
    Serial.println("Moving to center...");
    rightLimitHit_ = false;
    startTime = millis();
    motor_.setSpeed(-CALIBRATION_SPEED / 2 * STEERING_MOTOR_INVERT);
    while (encoderCount_ > centerEncoderCount_ + 10 &&
           (millis() - startTime < CALIBRATION_TIMEOUT_MS)) {
      delay(10);
    }
    motor_.stop();

    // Reset encoder to 0 at center
    encoderCount_ = 0;
    centerEncoderCount_ = 0;
    saveCalibration();

    calibrationState_ = CALIBRATED;
    emergencyStop_ = false;
    Serial.println("Calibration complete!");
    return true;
  }

  /**
   * @brief Get calibration state
   */
  CalibrationState getCalibrationState() const { return calibrationState_; }

  /**
   * @brief Check if emergency stop is active
   */
  bool isEmergencyStop() const { return emergencyStop_; }

  /**
   * @brief Clear emergency stop (only if no limit switch is physically pressed)
   */
  void clearEmergencyStop() {
    if (!leftLimitHit_ && !rightLimitHit_) {
      emergencyStop_ = false;
    }
  }

  /**
   * @brief Force clear emergency stop regardless of limit switch state
   * Use this after physically moving steering back from limit
   */
  void forceResetEmergencyStop() {
    emergencyStop_ = false;
    leftLimitHit_ = false;
    rightLimitHit_ = false;
  }

  /**
   * @brief Clear limit switch flags after calibration complete
   * Calibration leaves these flags set, but they should be cleared
   * before normal operation begins
   */
  void clearLimitSwitchFlags() {
    leftLimitHit_ = false;
    rightLimitHit_ = false;
  }

  /**
   * @brief Set startup grace period flag (suppresses emergency stop triggers)
   * Call this right after micro-ROS initializes
   */
  void setStartupGracePeriod(bool enabled) {
    startupGracePeriodEnabled_ = enabled;
  }

  /**
   * @brief Get encoder count
   */
  long getEncoderCount() const { return encoderCount_; }

  /**
   * @brief Get steering angle based on encoder position
   * This is the actual angle calculated from encoder pulses
   *
   * @return float Steering angle in degrees
   */
  float getSteeringAngleFromEncoder() const {
    return (float)encoderCount_ / PULSES_PER_DEGREE;
  }

  /**
   * @brief Get target steering angle to reach
   */
  float getTargetSteeringAngle() const { return targetAngle_; }

  /**
   * @brief Get left limit switch state
   */
  bool getLeftLimitState() const { return leftLimitHit_; }

  /**
   * @brief Get right limit switch state
   */
  bool getRightLimitState() const { return rightLimitHit_; }

  /**
   * @brief Get calibration state as string
   */
  const char *getCalibrationStateString() const {
    switch (calibrationState_) {
    case NOT_CALIBRATED:
      return "NOT_CALIBRATED";
    case CALIBRATING:
      return "CALIBRATING";
    case CALIBRATED:
      return "CALIBRATED";
    case CALIBRATION_ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
    }
  }

  /**
   * @brief Check if steering is back in safe zone (away from limit switches)
   * Used to auto-clear emergency stop after user moves steering back
   * Safe zone is considered center ±8 degrees (400 pulses)
   */
  bool isInSafeZone() const {
    // Safe zone: -400 to +400 pulses from center
    return (encoderCount_ >= -400 && encoderCount_ <= 400);
  }

  /**
   * @brief Auto-clear emergency stop if steering is in safe zone
   * Call this regularly from main loop to allow recovery
   */
  void attemptAutoClearEmergencyStop() {
    if (emergencyStop_ && !startupGracePeriodEnabled_) {
      // Only auto-clear if steering is well away from limits
      if (isInSafeZone()) {
        leftLimitHit_ = false;
        rightLimitHit_ = false;
        emergencyStop_ = false;
        Serial.println("✓ Emergency stop auto-cleared (steering in safe zone)");
      }
    }
  }

  // Static instance pointer for ISR access
  static SteeringController *instance_;

  /**
   * @brief Clear saved calibration to force fresh calibration
   */
  void clearCalibration() {
    preferences_.begin("steering", false);
    preferences_.clear();
    preferences_.end();
    calibrationState_ = NOT_CALIBRATED;
    centerEncoderCount_ = 0;
    Serial.println("Calibration cleared - will recalibrate on next boot");
  }

private:
  MotorDriver &motor_;
  volatile long encoderCount_;
  long centerEncoderCount_;
  double targetAngle_;         // Raw target from setTargetAngle()
  double smoothedTargetAngle_; // Rate-limited target fed to PID
  double currentAngle_;        // Filtered current angle
  double filteredAngle_;       // EMA filtered angle
  double pidOutput_;
  CalibrationState calibrationState_;
  volatile bool leftLimitHit_;
  volatile bool rightLimitHit_;
  volatile bool emergencyStop_;
  volatile bool
      startupGracePeriodEnabled_; // Suppress emergency stop during startup
  volatile unsigned long lastLeftLimitTime_;  // Debounce timestamp
  volatile unsigned long lastRightLimitTime_; // Debounce timestamp
  unsigned long lastUpdateTime_; // For rate limiting dt calculation
  PID pid_;
  Preferences preferences_;
  static const unsigned long LIMIT_DEBOUNCE_MS = 50; // 50ms debounce

  /**
   * @brief Update current angle from encoder count with EMA filter
   * EMA filter reduces noise and jitter in angle readings
   */
  void updateCurrentAngle() {
    // Raw angle from encoder
    float rawAngle = (float)encoderCount_ / PULSES_PER_DEGREE;

    // EMA (Exponential Moving Average) filter
    // Alpha = 0.3 gives good smoothing while remaining responsive
    const float EMA_ALPHA = 0.3f;
    filteredAngle_ = EMA_ALPHA * rawAngle + (1.0f - EMA_ALPHA) * filteredAngle_;

    currentAngle_ = filteredAngle_;
  }

  /**
   * @brief Encoder interrupt handler
   * IMPORTANT: Keep ISR as lightweight as possible!
   * Do NOT call heavy functions like updateCurrentAngle() or millis()
   */
  void handleEncoderISR() {
    bool aState = digitalRead(ENCODER_A_PIN);
    bool bState = digitalRead(ENCODER_B_PIN);

    // Quadrature decoding
    if (aState == bState) {
      encoderCount_++;
    } else {
      encoderCount_--;
    }

    // Check soft limits using encoder count directly (no floating point math)
    // PULSES_PER_DEGREE = 50, so ±10 degrees = ±500 pulses
    // Add 20% margin = ±600 pulses
    if (encoderCount_ < -600 || encoderCount_ > 600) {
      emergencyStop_ = true;
    }
  }

  /**
   * @brief Left limit switch interrupt handler with debouncing
   */
  void handleLeftLimitISR() {
    // Skip if we're in startup grace period (suppresses spurious triggers)
    if (startupGracePeriodEnabled_) {
      return;
    }

    unsigned long currentTime = millis();
    // Only process if debounce time has passed
    if (currentTime - lastLeftLimitTime_ >= LIMIT_DEBOUNCE_MS) {
      // Check if switch is triggered (active HIGH for NC switches, active LOW
      // for NO switches)
#if LIMIT_SWITCH_ACTIVE_HIGH
      if (digitalRead(LIMIT_SWITCH_LEFT) == HIGH) {
#else
      if (digitalRead(LIMIT_SWITCH_LEFT) == LOW) {
#endif
        leftLimitHit_ = true;
        emergencyStop_ = true;
        motor_.stop();
      }
      lastLeftLimitTime_ = currentTime;
    }
  }

  /**
   * @brief Right limit switch interrupt handler with debouncing
   */
  void handleRightLimitISR() {
    // Skip if we're in startup grace period (suppresses spurious triggers)
    if (startupGracePeriodEnabled_) {
      return;
    }

    unsigned long currentTime = millis();
    // Only process if debounce time has passed
    if (currentTime - lastRightLimitTime_ >= LIMIT_DEBOUNCE_MS) {
      // Check if switch is triggered (active HIGH for NC switches, active LOW
      // for NO switches)
#if LIMIT_SWITCH_ACTIVE_HIGH
      if (digitalRead(LIMIT_SWITCH_RIGHT) == HIGH) {
#else
      if (digitalRead(LIMIT_SWITCH_RIGHT) == LOW) {
#endif
        rightLimitHit_ = true;
        emergencyStop_ = true;
        motor_.stop();
      }
      lastRightLimitTime_ = currentTime;
    }
  }

  /**
   * @brief Save calibration to non-volatile memory
   */
  void saveCalibration() {
    preferences_.begin("steering", false);
    preferences_.putLong("centerCount", centerEncoderCount_);
    preferences_.putBool("calibrated", true);
    preferences_.end();
  }

  /**
   * @brief Load calibration from non-volatile memory
   */
  void loadCalibration() {
    preferences_.begin("steering", true);
    bool isCalibrated = preferences_.getBool("calibrated", false);
    if (isCalibrated) {
      centerEncoderCount_ = preferences_.getLong("centerCount", 0);
      calibrationState_ = CALIBRATED;
    }
    preferences_.end();
  }
};

// Initialize static instance pointer
SteeringController *SteeringController::instance_ = nullptr;

#endif // STEERING_CONTROLLER_H
