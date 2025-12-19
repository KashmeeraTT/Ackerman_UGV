// ============================================================================
// MPU6050 IMU Driver
// ============================================================================
// Driver for MPU6050 6-DOF IMU with complementary filter
// ============================================================================

#ifndef IMU_H
#define IMU_H

#include "config.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

class IMU {
public:
  IMU();

  /**
   * @brief Initialize IMU
   * @return true if successful
   */
  bool begin();

  /**
   * @brief Check if IMU is initialized
   */
  bool isInitialized() const { return initialized_; }

  /**
   * @brief Update sensor readings and orientation estimate
   * Call at regular intervals (recommended: 50 Hz)
   */
  void update();

  // Accelerometer (m/s²) - ROS frame (X=forward, Y=left, Z=up)
  float getAccelX() const { return accelX_; }
  float getAccelY() const { return accelY_; }
  float getAccelZ() const { return accelZ_; }

  // Gyroscope (rad/s) - ROS frame
  float getGyroX() const { return gyroX_; }
  float getGyroY() const { return gyroY_; }
  float getGyroZ() const { return gyroZ_; }

  // Orientation (degrees)
  float getPitch() const { return pitch_; }
  float getRoll() const { return roll_; }
  float getYaw() const { return yaw_; }

  // Temperature (°C)
  float getTemperature() const { return temperature_; }

  /**
   * @brief Reset yaw to zero
   */
  void resetYaw() { yaw_ = 0.0f; }

  /**
   * @brief Get quaternion orientation for ROS2 Imu message
   * @param qw, qx, qy, qz Quaternion components (output)
   */
  void getQuaternion(float &qw, float &qx, float &qy, float &qz) const;

private:
  Adafruit_MPU6050 mpu_;
  bool initialized_;

  // Sensor readings (ROS frame)
  float accelX_, accelY_, accelZ_;
  float gyroX_, gyroY_, gyroZ_;
  float temperature_;

  // Filtered orientation (degrees)
  float pitch_, roll_, yaw_;

  // Timing
  unsigned long lastUpdateTime_;
};

#endif // IMU_H
