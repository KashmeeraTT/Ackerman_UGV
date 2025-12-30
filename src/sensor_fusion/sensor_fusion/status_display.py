#!/usr/bin/env python3
"""
Sensor Fusion Status Display

Shows side-by-side comparison of:
- Left: What is COMMANDED (cmd_vel, steering intent)
- Right: What is PERCEIVED (IMU, VSLAM, steering feedback)

Run: ros2 run sensor_fusion status_display.py
"""

import rclpy
from rclpy.node import Node
import math
import os
import sys

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


class StatusDisplay(Node):
    def __init__(self):
        super().__init__('sensor_fusion_status')
        
        # Commanded state
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.steering_commanded = False
        
        # Perceived state
        self.vslam_x = 0.0
        self.vslam_y = 0.0
        self.vslam_yaw = 0.0
        self.vslam_vx = 0.0
        
        self.steering_feedback = 0.0
        
        self.camera_imu_accel = 0.0
        self.camera_imu_gyro = 0.0
        
        self.body_imu_accel = 0.0
        self.body_imu_gyro = 0.0
        
        self.health_score = 0.0
        
        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Odometry, '/odom', self.vslam_cb, 10)
        self.create_subscription(Float32, '/ugv/steering_angle', self.steering_cb, 10)
        self.create_subscription(Imu, '/camera/gyro_accel/sample', self.camera_imu_cb, 10)
        self.create_subscription(Imu, '/ugv/imu', self.body_imu_cb, 10)
        self.create_subscription(Float32, '/sensor_fusion/health_score', self.health_cb, 10)
        
        # Display timer (5 Hz)
        self.create_timer(0.2, self.display)
        
    def cmd_vel_cb(self, msg):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z
        self.steering_commanded = abs(msg.angular.z) > 0.05
        
    def vslam_cb(self, msg):
        self.vslam_x = msg.pose.pose.position.x
        self.vslam_y = msg.pose.pose.position.y
        self.vslam_vx = msg.twist.twist.linear.x
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.vslam_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        
    def steering_cb(self, msg):
        self.steering_feedback = msg.data
        
    def camera_imu_cb(self, msg):
        self.camera_imu_accel = math.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )
        self.camera_imu_gyro = math.sqrt(
            msg.angular_velocity.x**2 +
            msg.angular_velocity.y**2 +
            msg.angular_velocity.z**2
        )
        
    def body_imu_cb(self, msg):
        self.body_imu_accel = math.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )
        self.body_imu_gyro = math.sqrt(
            msg.angular_velocity.x**2 +
            msg.angular_velocity.y**2 +
            msg.angular_velocity.z**2
        )
        
    def health_cb(self, msg):
        self.health_score = msg.data

    def display(self):
        # Clear screen
        os.system('clear' if os.name == 'posix' else 'cls')
        
        # Determine motion state
        if abs(self.cmd_linear) < 0.02:
            motion = "STATIONARY"
        elif self.steering_commanded:
            motion = "MOVING+STEERING"
        else:
            motion = "MOVING STRAIGHT"
            
        # Check consistency
        vslam_moving = abs(self.vslam_vx) > 0.05
        cmd_moving = abs(self.cmd_linear) > 0.02
        consistent = (vslam_moving == cmd_moving)
        
        print("=" * 70)
        print("         SENSOR FUSION STATUS DISPLAY")
        print("=" * 70)
        print()
        print(f"  {'COMMANDED':^30} │ {'PERCEIVED':^30}")
        print(f"  {'-'*30} │ {'-'*30}")
        print()
        print(f"  Linear:  {self.cmd_linear:+6.3f} m/s          │ VSLAM Vel: {self.vslam_vx:+6.3f} m/s")
        print(f"  Angular: {self.cmd_angular:+6.3f} rad/s        │ VSLAM Yaw: {math.degrees(self.vslam_yaw):+6.1f}°")
        print(f"  Steer Cmd: {'YES' if self.steering_commanded else 'NO':^6}           │ Steer Angle: {self.steering_feedback:+6.2f}°")
        print()
        print(f"  {'-'*30} │ {'-'*30}")
        print(f"  Expected State: {motion:^13} │ Camera IMU Accel: {self.camera_imu_accel:6.2f} m/s²")
        print(f"                                │ Camera IMU Gyro:  {self.camera_imu_gyro:6.3f} rad/s")
        print(f"                                │ Body IMU Accel:   {self.body_imu_accel:6.2f} m/s²")
        print(f"                                │ Body IMU Gyro:    {self.body_imu_gyro:6.3f} rad/s")
        print()
        print(f"  {'─'*30} ┼ {'─'*30}")
        print(f"  VSLAM Position: ({self.vslam_x:+6.2f}, {self.vslam_y:+6.2f}) m")
        print()
        print(f"  ┌{'─'*66}┐")
        health_bar_len = int(self.health_score / 100 * 50)
        health_color = "🟢" if self.health_score > 70 else "🟡" if self.health_score > 40 else "🔴"
        print(f"  │ Health: {health_color} {self.health_score:5.1f}% │{'█'*health_bar_len}{'░'*(50-health_bar_len)}│")
        print(f"  └{'─'*66}┘")
        print()
        consistency_icon = "✓" if consistent else "✗"
        print(f"  Consistency Check: {consistency_icon} {'MATCH' if consistent else 'MISMATCH - CMD vs VSLAM'}")
        print()
        print("  Press Ctrl+C to exit")


def main(args=None):
    rclpy.init(args=args)
    node = StatusDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
