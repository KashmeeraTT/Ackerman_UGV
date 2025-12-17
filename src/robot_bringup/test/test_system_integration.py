#!/usr/bin/env python3
"""
Integration tests for robot system.

These tests verify the system is functioning correctly at runtime.
Run these tests after launching the system.

Usage:
    ros2 run robot_bringup test_system_integration.py
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import sys


from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class SystemIntegrationTest(Node):
    """Integration tests for verifying system functionality."""

    def __init__(self):
        super().__init__('system_integration_test')
        
        self.test_results = {}
        self.tests_complete = False
        
        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Topic tracking
        self.odom_received = False
        self.scan_received = False
        self.health_received = False
        self.health_status = None
        
        # QoS for sensor topics (Best Effort)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.health_sub = self.create_subscription(
            String, '/robot/health', self.health_callback, 10)
        
        # Start tests after a longer delay for system to fully initialize
        self.create_timer(5.0, self.run_tests)
        
        self.get_logger().info('Integration Test Node started, waiting 5s for system...')

    def odom_callback(self, msg):
        self.odom_received = True

    def scan_callback(self, msg):
        self.scan_received = True

    def health_callback(self, msg):
        self.health_received = True
        self.health_status = msg.data

    def run_tests(self):
        """Run all integration tests."""
        if self.tests_complete:
            return
            
        self.tests_complete = True
        self.get_logger().info('='*50)
        self.get_logger().info('Running Integration Tests')
        self.get_logger().info('='*50)
        
        # Test TF frames
        self.test_tf_frame('map', 'odom', 'Map to Odom TF')
        self.test_tf_frame('odom', 'base_link', 'Odom to Base Link TF')
        self.test_tf_frame('base_link', 'camera_link', 'Base Link to Camera TF')
        
        # Test topics
        self.test_results['Odometry Topic'] = 'PASS' if self.odom_received else 'FAIL'
        self.test_results['LaserScan Topic'] = 'PASS' if self.scan_received else 'FAIL'
        self.test_results['Health Monitor'] = 'PASS' if self.health_received else 'FAIL'
        
        # Print results
        self.get_logger().info('')
        self.get_logger().info('Test Results:')
        self.get_logger().info('-'*50)
        
        all_passed = True
        for test_name, result in self.test_results.items():
            status_icon = '✓' if result == 'PASS' else '✗'
            self.get_logger().info(f'  {status_icon} {test_name}: {result}')
            if result == 'FAIL':
                all_passed = False
        
        self.get_logger().info('-'*50)
        if all_passed:
            self.get_logger().info('All tests PASSED!')
        else:
            self.get_logger().warn('Some tests FAILED!')
        
        self.get_logger().info('='*50)
        
        # Exit with appropriate code
        if all_passed:
            sys.exit(0)
        else:
            sys.exit(1)

    def test_tf_frame(self, parent_frame, child_frame, test_name):
        """Test if a TF transform is available."""
        try:
            transform = self.tf_buffer.lookup_transform(
                parent_frame, child_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
            self.test_results[test_name] = 'PASS'
        except Exception as e:
            self.test_results[test_name] = f'FAIL ({str(e)[:30]})'


def main():
    rclpy.init()
    node = SystemIntegrationTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
