// ============================================================================
// micro-ROS Bridge Implementation
// ============================================================================

#include "comms/ros_bridge.h"

// Static callback pointer
CmdVelCallback RosBridge::userCallback_ = nullptr;

RosBridge::RosBridge() : connected_(false), lastPingTime_(0) {}

void RosBridge::cmdVelCallbackWrapper(const void *msg) {
  const geometry_msgs__msg__Twist *twist =
      static_cast<const geometry_msgs__msg__Twist *>(msg);

  if (userCallback_) {
    userCallback_(twist->linear.x, twist->angular.z);
  }
}

bool RosBridge::begin(CmdVelCallback callback) {
  userCallback_ = callback;

  // Setup serial transport
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator_ = rcl_get_default_allocator();

  // Initialize support
  if (rclc_support_init(&support_, 0, NULL, &allocator_) != RCL_RET_OK) {
    return false;
  }

  // Create node
  if (rclc_node_init_default(&node_, NODE_NAME, "", &support_) != RCL_RET_OK) {
    return false;
  }

  // Create cmd_vel subscriber
  if (rclc_subscription_init_default(
          &cmdVelSub_, &node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
          TOPIC_CMD_VEL) != RCL_RET_OK) {
    return false;
  }

  // Create publishers
  if (rclc_publisher_init_default(
          &heartbeatPub_, &node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
          TOPIC_HEARTBEAT) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &steeringPub_, &node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
          TOPIC_STEERING_ANGLE) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &imuPub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
          TOPIC_IMU) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &statusPub_, &node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          TOPIC_STATUS) != RCL_RET_OK) {
    return false;
  }

  // Create executor
  if (rclc_executor_init(&executor_, &support_.context, 1, &allocator_) !=
      RCL_RET_OK) {
    return false;
  }

  // Add subscription to executor
  if (rclc_executor_add_subscription(&executor_, &cmdVelSub_, &cmdVelMsg_,
                                     &cmdVelCallbackWrapper,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  // Initialize messages
  heartbeatMsg_.data = true;
  steeringMsg_.data = 0.0f;

  statusMsg_.data.data = statusBuffer_;
  statusMsg_.data.capacity = sizeof(statusBuffer_);

  // Initialize IMU message
  imuMsg_.header.frame_id.data = const_cast<char *>("imu_link");
  imuMsg_.header.frame_id.size = 8;
  imuMsg_.header.frame_id.capacity = 9;

  // Set covariance values
  imuMsg_.orientation_covariance[0] = 0.01;
  imuMsg_.orientation_covariance[4] = 0.01;
  imuMsg_.orientation_covariance[8] = 0.05;
  imuMsg_.angular_velocity_covariance[0] = 0.001;
  imuMsg_.angular_velocity_covariance[4] = 0.001;
  imuMsg_.angular_velocity_covariance[8] = 0.001;
  imuMsg_.linear_acceleration_covariance[0] = 0.01;
  imuMsg_.linear_acceleration_covariance[4] = 0.01;
  imuMsg_.linear_acceleration_covariance[8] = 0.01;

  connected_ = true;
  return true;
}

void RosBridge::spin() {
  if (!connected_)
    return;
  rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
}

bool RosBridge::checkConnection() {
  // Skip connection check if never connected (speeds up boot)
  if (!connected_ && lastPingTime_ == 0) {
    return false;
  }

  unsigned long now = millis();

  // Only check every 5 seconds to minimize overhead
  if (now - lastPingTime_ < PING_INTERVAL_MS) {
    return connected_;
  }
  lastPingTime_ = now;

  // Quick ping with very short timeout (10ms, 1 attempt)
  if (rmw_uros_ping_agent(10, 1) != RMW_RET_OK) {
    if (connected_) {
      connected_ = false;
      Serial.println("micro-ROS agent disconnected!");

      // Cleanup old resources for potential reconnection
      rclc_executor_fini(&executor_);
      rcl_publisher_fini(&heartbeatPub_, &node_);
      rcl_publisher_fini(&steeringPub_, &node_);
      rcl_publisher_fini(&imuPub_, &node_);
      rcl_publisher_fini(&statusPub_, &node_);
      rcl_subscription_fini(&cmdVelSub_, &node_);
      rcl_node_fini(&node_);
      rclc_support_fini(&support_);
    }
    return false;
  }

  // Agent is available
  if (!connected_) {
    Serial.println("micro-ROS agent detected, attempting reconnection...");

    // Attempt to reinitialize everything
    if (begin(userCallback_)) {
      Serial.println("micro-ROS reconnected successfully!");
    } else {
      Serial.println("micro-ROS reconnection failed!");
      return false;
    }
  }
  return true;
}

void RosBridge::publishHeartbeat() {
  if (!connected_)
    return;
  heartbeatMsg_.data = true;
  rcl_publish(&heartbeatPub_, &heartbeatMsg_, NULL);
}

void RosBridge::publishSteeringAngle(float angle) {
  if (!connected_)
    return;
  steeringMsg_.data = angle;
  rcl_publish(&steeringPub_, &steeringMsg_, NULL);
}

void RosBridge::publishImu(float qw, float qx, float qy, float qz, float gyroX,
                           float gyroY, float gyroZ, float accelX, float accelY,
                           float accelZ) {
  if (!connected_)
    return;

  unsigned long now = millis();
  imuMsg_.header.stamp.sec = now / 1000;
  imuMsg_.header.stamp.nanosec = (now % 1000) * 1000000;

  imuMsg_.orientation.w = qw;
  imuMsg_.orientation.x = qx;
  imuMsg_.orientation.y = qy;
  imuMsg_.orientation.z = qz;

  imuMsg_.angular_velocity.x = gyroX;
  imuMsg_.angular_velocity.y = gyroY;
  imuMsg_.angular_velocity.z = gyroZ;

  imuMsg_.linear_acceleration.x = accelX;
  imuMsg_.linear_acceleration.y = accelY;
  imuMsg_.linear_acceleration.z = accelZ;

  rcl_publish(&imuPub_, &imuMsg_, NULL);
}

void RosBridge::publishStatus(const char *status) {
  if (!connected_)
    return;
  strncpy(statusBuffer_, status, sizeof(statusBuffer_) - 1);
  statusBuffer_[sizeof(statusBuffer_) - 1] = '\0';
  statusMsg_.data.size = strlen(statusBuffer_);
  rcl_publish(&statusPub_, &statusMsg_, NULL);
}
