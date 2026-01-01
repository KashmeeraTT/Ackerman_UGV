#!/usr/bin/env python3
"""
Adaptive Sensor Fusion Node

Monitors sensor health against expected robot motion and produces reliable odometry.
Uses cmd_vel and steering feedback to determine expected motion state,
then validates IMU readings against this expectation.

Visual SLAM remains the primary source - IMUs are optional enhancements.

IMPORTANT: Uses lazy subscriptions to avoid conflicts with camera initialization.
Camera topics are only subscribed after a 15-second delay.
"""

import math
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


class MotionState(Enum):
    """
    Expected motion state based on commands.
    Note: Ackermann UGV cannot spin - steering only works during forward/reverse motion.
    """
    STATIONARY = 0      # No motion commanded
    MOVING_STRAIGHT = 1 # Linear motion, no steering
    MOVING_STEERING = 2 # Linear motion with steering (arc motion)
    UNKNOWN = 3         # Not enough data


@dataclass
class SensorHealth:
    """Health metrics for a sensor"""
    healthy: bool = True
    score: float = 100.0  # 0-100%
    reason: str = ""
    last_update: Optional[Time] = None


class AdaptiveFusion(Node):
    """
    Adaptive sensor fusion with motion-based health monitoring.
    
    Monitors:
    - Visual SLAM odometry (/odom)
    - Camera IMU (/camera/gyro_accel/sample)
    - Body IMU (/ugv/imu)
    
    Uses expected motion from:
    - /cmd_vel (velocity commands)
    - /ugv/steering_angle (steering feedback)
    
    IMPORTANT: Uses lazy subscriptions - camera topics are only subscribed
    after startup delay to avoid conflicts with camera initialization.
    """

    def __init__(self):
        super().__init__('adaptive_fusion')
        
        # Parameters
        self.declare_parameter('stationary_threshold', 0.02)  # m/s and rad/s
        self.declare_parameter('stationary_timeout', 0.5)     # seconds to confirm stationary
        self.declare_parameter('imu_variance_threshold', 0.5) # m/s² variance when stationary
        self.declare_parameter('sensor_timeout', 0.5)         # seconds before sensor unhealthy
        self.declare_parameter('health_window', 1.0)          # seconds for health averaging
        self.declare_parameter('startup_delay', 15.0)         # seconds before subscribing to camera topics
        
        self.stationary_threshold = self.get_parameter('stationary_threshold').value
        self.stationary_timeout = self.get_parameter('stationary_timeout').value
        self.imu_variance_threshold = self.get_parameter('imu_variance_threshold').value
        self.sensor_timeout = self.get_parameter('sensor_timeout').value
        self.health_window = self.get_parameter('health_window').value
        self.startup_delay = self.get_parameter('startup_delay').value
        
        # Motion state tracking
        self.motion_state = MotionState.UNKNOWN
        self.last_cmd_vel: Optional[Twist] = None
        self.last_cmd_time: Optional[Time] = None
        self.stationary_start: Optional[Time] = None
        
        # Steering tracking
        self.steering_angle = 0.0          # Current angle in degrees
        self.last_steering_angle = 0.0     # Previous angle for rate calculation
        self.last_steering_time: Optional[Time] = None
        self.steering_rate = 0.0           # deg/s - rate of change
        self.is_actively_steering = False  # True when steering motor is moving
        
        # Sensor health
        self.vslam_health = SensorHealth()
        self.camera_imu_health = SensorHealth()
        self.body_imu_health = SensorHealth()
        
        # IMU data buffers for variance calculation
        self.camera_imu_buffer = deque(maxlen=50)  # ~1 second at 50Hz
        self.body_imu_buffer = deque(maxlen=50)
        
        # Last VSLAM pose for drift detection
        self.last_vslam_pose = None
        self.last_vslam_time: Optional[Time] = None
        
        # QoS profiles
        # Default reliable QoS
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Best effort QoS for camera topics (matches camera driver)
        self.best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers (created immediately)
        self.health_pub = self.create_publisher(
            Float32, '/sensor_fusion/health_score', 10)
        
        # Forward healthy IMU data (optional - for debugging/monitoring)
        self.camera_imu_healthy_pub = self.create_publisher(
            Imu, '/sensor_fusion/camera_imu_healthy', 10)
        self.body_imu_healthy_pub = self.create_publisher(
            Imu, '/sensor_fusion/body_imu_healthy', 10)
        
        # Lazy subscription tracking
        self.subscriptions_created = False
        self.camera_imu_sub = None
        
        # Create non-camera subscriptions immediately
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.steering_sub = self.create_subscription(
            Float32, '/ugv/steering_angle', self.steering_callback, 10)
        self.vslam_odom_sub = self.create_subscription(
            Odometry, '/odom', self.vslam_odom_callback, 10)
        self.body_imu_sub = self.create_subscription(
            Imu, '/ugv/imu', self.body_imu_callback, self.reliable_qos)
        
        # Delayed subscription creation for camera topics
        self.get_logger().info(f'Adaptive Fusion: waiting {self.startup_delay}s before subscribing to camera topics...')
        self.startup_timer = self.create_timer(self.startup_delay, self._create_camera_subscriptions)
        
        # Health check timer
        self.health_timer = self.create_timer(0.1, self.health_check)  # 10Hz
        
        self.get_logger().info('Adaptive Sensor Fusion node starting (lazy subscription mode)')
        self.get_logger().info(f'  Stationary threshold: {self.stationary_threshold} m/s')
        self.get_logger().info(f'  IMU variance threshold: {self.imu_variance_threshold} m/s²')

    def _create_camera_subscriptions(self):
        """Create camera topic subscriptions after delay."""
        # Cancel the one-shot timer
        self.startup_timer.cancel()
        
        if self.subscriptions_created:
            return
            
        self.get_logger().info('Creating camera IMU subscription (BEST_EFFORT QoS)...')
        
        # Subscribe to camera IMU with BEST_EFFORT QoS to match camera driver
        self.camera_imu_sub = self.create_subscription(
            Imu, '/camera/gyro_accel/sample', 
            self.camera_imu_callback, 
            self.best_effort_qos
        )
        
        self.subscriptions_created = True
        self.get_logger().info('Adaptive Sensor Fusion fully initialized')

    def cmd_vel_callback(self, msg: Twist):
        """
        Track commanded velocity to determine expected motion state.
        For Ackermann UGV:
        - linear.x = forward/reverse speed
        - angular.z = steering command (converted to steering angle by ackermann_bridge)
        """
        self.last_cmd_vel = msg
        self.last_cmd_time = self.get_clock().now()
        
        # Track steering command from angular.z (intent to steer)
        # Non-zero angular.z means steering motor will be active
        steering_commanded = abs(msg.angular.z) > 0.05  # rad/s threshold
        
        # Ackermann: only linear.x determines if moving
        is_moving = abs(msg.linear.x) > self.stationary_threshold
        
        if not is_moving:
            if self.stationary_start is None:
                self.stationary_start = self.get_clock().now()
            elif self._seconds_since(self.stationary_start) > self.stationary_timeout:
                self.motion_state = MotionState.STATIONARY
        else:
            self.stationary_start = None
            # Check steering: either from cmd_vel.angular.z OR current steering angle
            if steering_commanded or abs(self.steering_angle) > 1.0:
                self.motion_state = MotionState.MOVING_STEERING
            else:
                self.motion_state = MotionState.MOVING_STRAIGHT
        
        # Update actively_steering based on command as well
        if steering_commanded:
            self.is_actively_steering = True

    def steering_callback(self, msg: Float32):
        """
        Track steering angle and rate of change.
        - steering_angle: current wheel angle (degrees)
        - steering_rate: how fast steering is changing (deg/s)
        - is_actively_steering: True if steering motor is moving
        """
        now = self.get_clock().now()
        new_angle = msg.data
        
        # Calculate rate of change
        if self.last_steering_time is not None:
            dt = self._time_diff(now, self.last_steering_time)
            if dt > 0.01:
                self.steering_rate = abs(new_angle - self.last_steering_angle) / dt
                # If changing faster than 5 deg/s, steering motor is active
                self.is_actively_steering = self.steering_rate > 5.0
        
        self.last_steering_angle = self.steering_angle
        self.steering_angle = new_angle
        self.last_steering_time = now

    def vslam_odom_callback(self, msg: Odometry):
        """Track VSLAM odometry for health monitoring"""
        now = self.get_clock().now()
        self.vslam_health.last_update = now
        
        # Check for sudden jumps (potential tracking issues)
        if self.last_vslam_pose is not None and self.last_vslam_time is not None:
            dt = self._time_diff(now, self.last_vslam_time)
            if dt > 0.01:  # Minimum time delta
                dx = msg.pose.pose.position.x - self.last_vslam_pose[0]
                dy = msg.pose.pose.position.y - self.last_vslam_pose[1]
                distance = math.sqrt(dx*dx + dy*dy)
                velocity = distance / dt
                
                # If velocity seems too high for expected motion, flag it
                if self.motion_state == MotionState.STATIONARY and velocity > 0.1:
                    self.vslam_health.score = max(50.0, self.vslam_health.score - 10)
                    self.vslam_health.reason = "Motion detected while stationary"
                else:
                    # Gradually restore health
                    self.vslam_health.score = min(100.0, self.vslam_health.score + 5)
                    self.vslam_health.reason = ""
        
        self.last_vslam_pose = (msg.pose.pose.position.x, 
                                msg.pose.pose.position.y,
                                msg.pose.pose.position.z)
        self.last_vslam_time = now
        
        self.vslam_health.healthy = (self.vslam_health.score > 50)

    def camera_imu_callback(self, msg: Imu):
        """Process camera IMU and check health"""
        now = self.get_clock().now()
        self.camera_imu_health.last_update = now
        
        # Store acceleration magnitude for variance calculation
        accel_mag = math.sqrt(
            msg.linear_acceleration.x**2 + 
            msg.linear_acceleration.y**2 + 
            msg.linear_acceleration.z**2
        )
        # Remove gravity (~9.8 m/s²) to get motion component
        accel_motion = abs(accel_mag - 9.81)
        self.camera_imu_buffer.append(accel_motion)
        
        # Update health based on motion state
        self._update_imu_health(self.camera_imu_health, self.camera_imu_buffer, "Camera IMU")
        
        # Forward if healthy
        if self.camera_imu_health.healthy:
            self.camera_imu_healthy_pub.publish(msg)

    def body_imu_callback(self, msg: Imu):
        """Process body IMU and check health"""
        now = self.get_clock().now()
        self.body_imu_health.last_update = now
        
        # Store acceleration magnitude for variance calculation
        accel_mag = math.sqrt(
            msg.linear_acceleration.x**2 + 
            msg.linear_acceleration.y**2 + 
            msg.linear_acceleration.z**2
        )
        accel_motion = abs(accel_mag - 9.81)
        self.body_imu_buffer.append(accel_motion)
        
        # Update health based on motion state
        self._update_imu_health(self.body_imu_health, self.body_imu_buffer, "Body IMU")
        
        # Forward if healthy
        if self.body_imu_health.healthy:
            self.body_imu_healthy_pub.publish(msg)

    def _update_imu_health(self, health: SensorHealth, buffer: deque, name: str):
        """Update IMU health based on variance vs expected motion"""
        if len(buffer) < 10:
            return  # Not enough data
        
        # Calculate variance
        mean_accel = sum(buffer) / len(buffer)
        variance = sum((x - mean_accel)**2 for x in buffer) / len(buffer)
        
        if self.motion_state == MotionState.STATIONARY:
            # When stationary, IMU should show low variance
            if variance > self.imu_variance_threshold:
                health.score = max(0.0, health.score - 5)
                health.reason = f"High variance ({variance:.3f}) while stationary"
                health.healthy = False
            else:
                health.score = min(100.0, health.score + 2)
                health.reason = ""
                health.healthy = True
        else:
            # When moving, some variance is expected
            if variance > self.imu_variance_threshold * 5:
                health.score = max(30.0, health.score - 3)
                health.reason = f"Excessive variance ({variance:.3f})"
            else:
                health.score = min(100.0, health.score + 2)
                health.reason = ""
            health.healthy = (health.score > 50)

    def health_check(self):
        """Periodic health check for all sensors"""
        now = self.get_clock().now()
        
        # Check for sensor timeouts
        for name, health in [("VSLAM", self.vslam_health),
                             ("Camera IMU", self.camera_imu_health),
                             ("Body IMU", self.body_imu_health)]:
            if health.last_update is not None:
                age = self._time_diff(now, health.last_update)
                if age > self.sensor_timeout:
                    health.healthy = False
                    health.score = max(0.0, health.score - 10)
                    health.reason = f"Timeout ({age:.1f}s)"
        
        # Publish overall health score
        overall = (self.vslam_health.score * 0.5 + 
                   self.camera_imu_health.score * 0.3 +
                   self.body_imu_health.score * 0.2)
        
        health_msg = Float32()
        health_msg.data = overall
        self.health_pub.publish(health_msg)
        
        # Log status periodically (every 5 seconds)
        if not hasattr(self, '_last_log') or self._seconds_since(self._last_log) > 5.0:
            self._last_log = now
            cam_status = "pending" if not self.subscriptions_created else f"{self.camera_imu_health.score:.0f}%"
            self.get_logger().info(
                f"Health: VSLAM={self.vslam_health.score:.0f}% "
                f"CamIMU={cam_status} "
                f"BodyIMU={self.body_imu_health.score:.0f}% "
                f"State={self.motion_state.name}"
            )

    def _seconds_since(self, past_time: Time) -> float:
        """Calculate seconds elapsed since a past time"""
        now = self.get_clock().now()
        return self._time_diff(now, past_time)

    def _time_diff(self, t1: Time, t2: Time) -> float:
        """Calculate time difference in seconds"""
        diff = t1 - t2
        return diff.nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Only shutdown if context is still valid (prevents race condition error)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

