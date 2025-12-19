// ============================================================================
// MPU6050 IMU Driver Implementation
// ============================================================================

#include "drivers/imu.h"

IMU::IMU()
    : initialized_(false), accelX_(0), accelY_(0), accelZ_(0), gyroX_(0),
      gyroY_(0), gyroZ_(0), temperature_(0), pitch_(0), roll_(0), yaw_(0),
      lastUpdateTime_(0) {}

bool IMU::begin() {
  if (!mpu_.begin()) {
    Serial.println("Failed to find MPU6050!");
    return false;
  }

  // Configure sensor ranges
  mpu_.setAccelerometerRange(IMU_ACCEL_RANGE);
  mpu_.setGyroRange(IMU_GYRO_RANGE);
  mpu_.setFilterBandwidth(IMU_FILTER_BW);

  Serial.println("MPU6050 initialized");
  initialized_ = true;
  lastUpdateTime_ = millis();
  return true;
}

void IMU::update() {
  if (!initialized_)
    return;

  // Get sensor events
  sensors_event_t accel, gyro, temp;
  mpu_.getEvent(&accel, &gyro, &temp);

  // Remap axes from IMU frame to ROS body frame (REP-103)
  // IMU mounting: X=Left, Y=Back, Z=Up
  // ROS convention: X=Forward, Y=Left, Z=Up
  // Transform: ROS_X = -IMU_Y, ROS_Y = +IMU_X, ROS_Z = +IMU_Z
  accelX_ = -accel.acceleration.y;
  accelY_ = accel.acceleration.x;
  accelZ_ = accel.acceleration.z;

  gyroX_ = -gyro.gyro.y;
  gyroY_ = gyro.gyro.x;
  gyroZ_ = gyro.gyro.z;

  temperature_ = temp.temperature;

  // Calculate time delta
  unsigned long now = millis();
  float dt = (now - lastUpdateTime_) / 1000.0f;
  lastUpdateTime_ = now;

  // Complementary filter for pitch and roll
  float accelPitch =
      atan2(-accelX_, sqrt(accelY_ * accelY_ + accelZ_ * accelZ_)) * RAD_TO_DEG;
  float accelRoll = atan2(accelY_, accelZ_) * RAD_TO_DEG;

  float gyroPitchRate = gyroY_ * RAD_TO_DEG;
  float gyroRollRate = gyroX_ * RAD_TO_DEG;
  float gyroYawRate = gyroZ_ * RAD_TO_DEG;

  // Complementary filter: trust gyro short-term, accel long-term
  pitch_ = IMU_COMPLEMENTARY_ALPHA * (pitch_ + gyroPitchRate * dt) +
           (1.0f - IMU_COMPLEMENTARY_ALPHA) * accelPitch;
  roll_ = IMU_COMPLEMENTARY_ALPHA * (roll_ + gyroRollRate * dt) +
          (1.0f - IMU_COMPLEMENTARY_ALPHA) * accelRoll;

  // Yaw from gyro only (no magnetometer)
  yaw_ += gyroYawRate * dt;

  // Normalize yaw to -180 to +180
  while (yaw_ > 180.0f)
    yaw_ -= 360.0f;
  while (yaw_ < -180.0f)
    yaw_ += 360.0f;
}

void IMU::getQuaternion(float &qw, float &qx, float &qy, float &qz) const {
  // Convert Euler angles to quaternion
  float pitchRad = pitch_ * DEG_TO_RAD / 2.0f;
  float rollRad = roll_ * DEG_TO_RAD / 2.0f;
  float yawRad = yaw_ * DEG_TO_RAD / 2.0f;

  float cy = cos(yawRad);
  float sy = sin(yawRad);
  float cp = cos(pitchRad);
  float sp = sin(pitchRad);
  float cr = cos(rollRad);
  float sr = sin(rollRad);

  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;
}
