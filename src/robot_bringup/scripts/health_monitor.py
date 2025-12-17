#!/usr/bin/env python3
"""
Health Monitor Node for Production Robot System.

This node monitors the health of critical robot systems and implements
safety behaviors when issues are detected.

Features:
- Monitors VSLAM odometry output
- Publishes diagnostic status
- Implements safe-stop on tracking loss
- Publishes simple health status topic
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import signal
import sys


class HealthMonitor(Node):
    """Monitor robot system health and implement safety behaviors."""

    def __init__(self):
        super().__init__('health_monitor')

        # Parameters
        self.declare_parameter('odom_timeout', 2.0)  # seconds before SLAM considered lost
        self.declare_parameter('safe_stop_enabled', True)
        self.declare_parameter('diagnostics_rate', 1.0)  # Hz
        
        self.odom_timeout = self.get_parameter('odom_timeout').value
        self.safe_stop_enabled = self.get_parameter('safe_stop_enabled').value
        diagnostics_rate = self.get_parameter('diagnostics_rate').value

        # State tracking
        self.last_odom_time = None
        self.slam_healthy = False
        self.safe_stop_triggered = False
        self.odom_count = 0
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        self._shutdown_requested = False

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Health status (simple string for easy monitoring)
        self.health_pub = self.create_publisher(String, '/robot/health', 10)
        
        # Diagnostics (standard ROS2 diagnostics)
        diag_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', diag_qos)

        # Timer for health checks
        timer_period = 1.0 / diagnostics_rate
        self.timer = self.create_timer(timer_period, self.health_check)

        self.get_logger().info('Health Monitor started')
        self.get_logger().info(f'  SLAM timeout: {self.odom_timeout}s')
        self.get_logger().info(f'  Safe-stop enabled: {self.safe_stop_enabled}')

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        self.get_logger().info('Shutdown signal received')
        self._shutdown_requested = True

    def odom_callback(self, msg: Odometry):
        """Track odometry messages to detect SLAM health."""
        self.last_odom_time = self.get_clock().now()
        self.odom_count += 1
        
        if not self.slam_healthy:
            self.slam_healthy = True
            self.safe_stop_triggered = False
            self.get_logger().info('SLAM tracking restored')

    def health_check(self):
        """Periodic health check and diagnostics publishing."""
        if self._shutdown_requested:
            return
            
        current_time = self.get_clock().now()
        
        # Check SLAM health
        if self.last_odom_time is not None:
            elapsed = (current_time - self.last_odom_time).nanoseconds / 1e9
            if elapsed > self.odom_timeout:
                if self.slam_healthy:
                    self.slam_healthy = False
                    self.get_logger().warn(
                        f'SLAM tracking lost (no odom for {elapsed:.1f}s)')
                    self._trigger_safe_stop()
        else:
            # Never received odom
            self.slam_healthy = False
        
        # Publish health status
        self._publish_health_status()
        self._publish_diagnostics()

    def _trigger_safe_stop(self):
        """Stop the robot when SLAM tracking is lost."""
        if not self.safe_stop_enabled:
            return
            
        if self.safe_stop_triggered:
            return  # Already stopped
            
        self.get_logger().warn('SAFE STOP triggered - sending zero velocity')
        self.safe_stop_triggered = True
        
        # Send zero velocity command
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.linear.z = 0.0
        stop_cmd.angular.x = 0.0
        stop_cmd.angular.y = 0.0
        stop_cmd.angular.z = 0.0
        
        # Publish multiple times to ensure it's received
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_cmd)

    def _publish_health_status(self):
        """Publish simple health status message."""
        msg = String()
        if self.slam_healthy:
            msg.data = 'OK'
        elif self.safe_stop_triggered:
            msg.data = 'STOPPED'
        else:
            msg.data = 'DEGRADED'
        self.health_pub.publish(msg)

    def _publish_diagnostics(self):
        """Publish standard ROS2 diagnostics."""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # SLAM status
        slam_status = DiagnosticStatus()
        slam_status.name = 'SLAM'
        slam_status.hardware_id = 'orbslam3'
        
        if self.slam_healthy:
            slam_status.level = DiagnosticStatus.OK
            slam_status.message = 'Tracking active'
        else:
            slam_status.level = DiagnosticStatus.ERROR
            slam_status.message = 'Tracking lost'
        
        slam_status.values = [
            KeyValue(key='odom_count', value=str(self.odom_count)),
            KeyValue(key='healthy', value=str(self.slam_healthy)),
            KeyValue(key='safe_stop_triggered', value=str(self.safe_stop_triggered)),
        ]
        
        diag_array.status.append(slam_status)
        self.diag_pub.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
    
    try:
        while rclpy.ok() and not node._shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Health Monitor shutting down')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
