// ============================================================================
// micro-ROS Bridge
// ============================================================================
// micro-ROS communication layer for ROS2 integration
// ============================================================================

#ifndef ROS_BRIDGE_H
#define ROS_BRIDGE_H

#include "config.h"
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>

// Callback function type
typedef void (*CmdVelCallback)(float linearX, float angularZ);

class RosBridge {
public:
  RosBridge();

  /**
   * @brief Initialize micro-ROS connection
   * @param callback Function to call on cmd_vel messages
   * @return true if successful
   */
  bool begin(CmdVelCallback callback);

  /**
   * @brief Spin executor to process callbacks
   */
  void spin();

  /**
   * @brief Publish heartbeat
   */
  void publishHeartbeat();

  /**
   * @brief Publish steering angle
   * @param angle Current steering angle in degrees
   */
  void publishSteeringAngle(float angle);

  /**
   * @brief Publish IMU data
   */
  void publishImu(float qw, float qx, float qy, float qz, float gyroX,
                  float gyroY, float gyroZ, float accelX, float accelY,
                  float accelZ);

  /**
   * @brief Publish status string
   * @param status Status message
   */
  void publishStatus(const char *status);

  /**
   * @brief Check if connected
   */
  bool isConnected() const { return connected_; }

  /**
   * @brief Check connection status with ping (call periodically)
   * @return true if still connected
   */
  bool checkConnection();

private:
  rcl_allocator_t allocator_;
  rclc_support_t support_;
  rcl_node_t node_;
  rclc_executor_t executor_;

  // Subscriber
  rcl_subscription_t cmdVelSub_;
  geometry_msgs__msg__Twist cmdVelMsg_;

  // Publishers
  rcl_publisher_t heartbeatPub_;
  rcl_publisher_t steeringPub_;
  rcl_publisher_t imuPub_;
  rcl_publisher_t statusPub_;

  std_msgs__msg__Bool heartbeatMsg_;
  std_msgs__msg__Float32 steeringMsg_;
  sensor_msgs__msg__Imu imuMsg_;
  std_msgs__msg__String statusMsg_;

  char statusBuffer_[256];
  bool connected_;
  unsigned long lastPingTime_;
  static const unsigned long PING_INTERVAL_MS =
      5000; // Check connection every 5 seconds

  static CmdVelCallback userCallback_;
  static void cmdVelCallbackWrapper(const void *msg);
};

#endif // ROS_BRIDGE_H
