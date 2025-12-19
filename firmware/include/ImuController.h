#ifndef IMU_CONTROLLER_H
#define IMU_CONTROLLER_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

/**
 * @brief IMU controller for GY87 module (MPU6050)
 *
 * Provides:
 * - Accelerometer readings (m/s²)
 * - Gyroscope readings (rad/s)
 * - Complementary filter for pitch/roll estimation
 * - Temperature reading
 */
class ImuController {
public:
  ImuController()
      : initialized_(false), pitch_(0.0), roll_(0.0), yaw_(0.0),
        lastUpdateTime_(0), complementaryAlpha_(0.98) {}

  /**
   * @brief Initialize IMU on I2C bus
   * @return true if initialization successful
   */
  bool begin() {
    if (!mpu_.begin()) {
      Serial.println("Failed to find MPU6050 chip!");
      return false;
    }

    // Configure sensor ranges
    mpu_.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("MPU6050 initialized!");
    Serial.print("  Accelerometer range: ±4G");
    Serial.println("  Gyroscope range: ±500°/s");

    initialized_ = true;
    lastUpdateTime_ = millis();
    return true;
  }

  /**
   * @brief Check if IMU is initialized
   */
  bool isInitialized() const { return initialized_; }

  /**
   * @brief Update sensor readings and orientation estimate
   * Call this at regular intervals (recommended: 50Hz / 20ms)
   *
   * NOTE: Axis remapping for physical mounting:
   * - IMU X points LEFT  → ROS +Y
   * - IMU Y points BACK  → ROS -X
   * - IMU Z points UP    → ROS +Z (unchanged)
   */
  void update() {
    if (!initialized_)
      return;

    // Get new sensor events (raw IMU frame)
    sensors_event_t accel, gyro, temp;
    mpu_.getEvent(&accel, &gyro, &temp);

    // Remap axes from IMU frame to ROS body frame (REP-103)
    // IMU mounting: X=Left, Y=Back, Z=Up
    // ROS convention: X=Forward, Y=Left, Z=Up
    // Transform: ROS_X = -IMU_Y, ROS_Y = +IMU_X, ROS_Z = +IMU_Z

    // Accelerometer (remapped to ROS frame)
    accelX_ = -accel.acceleration.y; // Forward = -Back
    accelY_ = accel.acceleration.x;  // Left = Left
    accelZ_ = accel.acceleration.z;  // Up = Up

    // Gyroscope (remapped to ROS frame, already in rad/s)
    gyroX_ = -gyro.gyro.y; // Roll rate around forward axis
    gyroY_ = gyro.gyro.x;  // Pitch rate around left axis
    gyroZ_ = gyro.gyro.z;  // Yaw rate around up axis

    temperature_ = temp.temperature;

    // Calculate time delta
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime_) / 1000.0f;
    lastUpdateTime_ = currentTime;

    // Complementary filter for pitch and roll (using remapped values)
    // Pitch: rotation around Y axis (left), positive = nose up
    float accelPitch =
        atan2(-accelX_, sqrt(accelY_ * accelY_ + accelZ_ * accelZ_)) *
        RAD_TO_DEG;
    // Roll: rotation around X axis (forward), positive = right side down
    float accelRoll = atan2(accelY_, accelZ_) * RAD_TO_DEG;

    // Gyro integration (smooth but drifts)
    float gyroPitchRate = gyroY_ * RAD_TO_DEG;
    float gyroRollRate = gyroX_ * RAD_TO_DEG;
    float gyroYawRate = gyroZ_ * RAD_TO_DEG;

    // Complementary filter: trust gyro short-term, accel long-term
    pitch_ = complementaryAlpha_ * (pitch_ + gyroPitchRate * dt) +
             (1.0 - complementaryAlpha_) * accelPitch;
    roll_ = complementaryAlpha_ * (roll_ + gyroRollRate * dt) +
            (1.0 - complementaryAlpha_) * accelRoll;

    // Yaw from gyro only (no magnetometer correction without HMC5883L)
    yaw_ += gyroYawRate * dt;

    // Normalize yaw to -180 to +180
    while (yaw_ > 180.0)
      yaw_ -= 360.0;
    while (yaw_ < -180.0)
      yaw_ += 360.0;
  }

  // Accelerometer getters (m/s²)
  float getAccelX() const { return accelX_; }
  float getAccelY() const { return accelY_; }
  float getAccelZ() const { return accelZ_; }

  // Gyroscope getters (rad/s)
  float getGyroX() const { return gyroX_; }
  float getGyroY() const { return gyroY_; }
  float getGyroZ() const { return gyroZ_; }

  // Orientation getters (degrees)
  float getPitch() const { return pitch_; }
  float getRoll() const { return roll_; }
  float getYaw() const { return yaw_; }

  // Temperature getter (°C)
  float getTemperature() const { return temperature_; }

  /**
   * @brief Reset yaw to zero (call when robot is facing forward)
   */
  void resetYaw() { yaw_ = 0.0; }

  /**
   * @brief Set complementary filter alpha (0.0-1.0)
   * Higher = more trust in gyro, lower = more trust in accelerometer
   */
  void setFilterAlpha(float alpha) {
    complementaryAlpha_ = constrain(alpha, 0.0f, 1.0f);
  }

  /**
   * @brief Get quaternion orientation for ROS2 Imu message
   * Converts Euler angles to quaternion (simplified - assumes small angles)
   */
  void getQuaternion(float &qw, float &qx, float &qy, float &qz) const {
    // Convert degrees to radians
    float pitchRad = pitch_ * DEG_TO_RAD / 2.0;
    float rollRad = roll_ * DEG_TO_RAD / 2.0;
    float yawRad = yaw_ * DEG_TO_RAD / 2.0;

    // Calculate quaternion components
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

private:
  Adafruit_MPU6050 mpu_;
  bool initialized_;

  // Raw sensor values
  float accelX_, accelY_, accelZ_; // m/s²
  float gyroX_, gyroY_, gyroZ_;    // rad/s
  float temperature_;              // °C

  // Filtered orientation (degrees)
  float pitch_, roll_, yaw_;

  // Timing
  unsigned long lastUpdateTime_;

  // Filter coefficient
  float complementaryAlpha_;
};

#endif // IMU_CONTROLLER_H
