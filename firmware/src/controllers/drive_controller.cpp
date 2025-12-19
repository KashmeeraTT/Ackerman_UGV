// ============================================================================
// Drive Controller Implementation
// ============================================================================

#include "controllers/drive_controller.h"

DriveController::DriveController(BTS7960 &motor)
    : motor_(motor), targetVelocity_(0), currentVelocity_(0),
      maxAccel_(MAX_ACCELERATION), maxVelocity_(MAX_VELOCITY_MPS) {}

void DriveController::begin() { motor_.stop(); }

void DriveController::update() {
  // Smooth acceleration
  float error = targetVelocity_ - currentVelocity_;

  if (abs(error) < maxAccel_) {
    currentVelocity_ = targetVelocity_;
  } else if (error > 0) {
    currentVelocity_ += maxAccel_;
  } else {
    currentVelocity_ -= maxAccel_;
  }

  // Convert to PWM and apply
  int16_t pwm = velocityToPWM(currentVelocity_);
  motor_.setSpeed(pwm);
}

void DriveController::setVelocity(float velocity) {
  targetVelocity_ = constrain(velocity, -maxVelocity_, maxVelocity_);
}

void DriveController::stop() {
  targetVelocity_ = 0;
  currentVelocity_ = 0;
  motor_.stop();
}

int16_t DriveController::velocityToPWM(float velocity) {
  // Very small velocities -> stop
  if (abs(velocity) < 0.05f) {
    return 0;
  }

  // Linear mapping
  float pwm = (velocity / maxVelocity_) * PWM_MAX_VALUE;

  // Ensure we meet deadband
  if (pwm > 0 && pwm < MOTOR_DEADBAND) {
    pwm = MOTOR_DEADBAND;
  } else if (pwm < 0 && pwm > -MOTOR_DEADBAND) {
    pwm = -MOTOR_DEADBAND;
  }

  return static_cast<int16_t>(constrain(pwm, -PWM_MAX_VALUE, PWM_MAX_VALUE));
}
