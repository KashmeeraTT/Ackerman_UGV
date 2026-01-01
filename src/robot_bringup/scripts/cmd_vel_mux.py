#!/usr/bin/env python3
"""
Command Velocity Priority Mux

Multiplexes multiple cmd_vel sources with priority:
1. Safety commands (highest priority) - from health_monitor
2. Navigation commands - from twist_to_ackermann bridge

Safety commands take precedence when:
- A safety stop is triggered (zero velocity with safety flag)
- Safety topic has been active within timeout period

This ensures the robot stops immediately when SLAM tracking is lost,
regardless of what Nav2 is commanding.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import signal


class CmdVelMux(Node):
    """Priority multiplexer for cmd_vel commands."""

    def __init__(self):
        super().__init__('cmd_vel_mux')
        
        # Parameters
        self.declare_parameter('safety_timeout', 1.0)  # seconds
        self.declare_parameter('nav_timeout', 0.5)     # seconds
        self.declare_parameter('output_rate', 20.0)    # Hz
        
        self.safety_timeout = self.get_parameter('safety_timeout').value
        self.nav_timeout = self.get_parameter('nav_timeout').value
        output_rate = self.get_parameter('output_rate').value
        
        # State tracking
        self.safety_cmd = Twist()  # Last safety command
        self.nav_cmd = Twist()     # Last navigation command
        self.safety_active = False
        self.last_safety_time = None
        self.last_nav_time = None
        self._shutdown_requested = False
        
        # Signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        # QoS profiles
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        # Safety input (from health_monitor)
        self.safety_sub = self.create_subscription(
            Twist, '/cmd_vel_safety', self.safety_callback, qos)
        
        # Safety active flag (explicit signal from health_monitor)
        self.safety_active_sub = self.create_subscription(
            Bool, '/cmd_vel_safety_active', self.safety_active_callback, qos)
        
        # Navigation input (from twist_to_ackermann)
        self.nav_sub = self.create_subscription(
            Twist, '/cmd_vel_nav_mux', self.nav_callback, qos)
        
        # Publisher - final output to ESP32
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        
        # Timer for output
        timer_period = 1.0 / output_rate
        self.timer = self.create_timer(timer_period, self.output_callback)
        
        self.get_logger().info('CmdVel Mux started')
        self.get_logger().info('  Safety input: /cmd_vel_safety')
        self.get_logger().info('  Nav input: /cmd_vel_nav_mux')
        self.get_logger().info('  Output: /cmd_vel')

    def _signal_handler(self, signum, frame):
        self._shutdown_requested = True

    def safety_callback(self, msg: Twist):
        """Receive safety command (highest priority)."""
        self.safety_cmd = msg
        self.last_safety_time = self.get_clock().now()

    def safety_active_callback(self, msg: Bool):
        """Explicit safety active flag from health_monitor."""
        if msg.data and not self.safety_active:
            self.get_logger().warn('Safety override ACTIVE - blocking nav commands')
        elif not msg.data and self.safety_active:
            self.get_logger().info('Safety override released - nav commands enabled')
        self.safety_active = msg.data

    def nav_callback(self, msg: Twist):
        """Receive navigation command."""
        self.nav_cmd = msg
        self.last_nav_time = self.get_clock().now()

    def output_callback(self):
        """Decide which command to output based on priority."""
        if self._shutdown_requested:
            return
            
        now = self.get_clock().now()
        output = Twist()
        source = "none"
        
        # Check if safety is active (explicit flag or recent command)
        safety_recent = False
        if self.last_safety_time is not None:
            elapsed = (now - self.last_safety_time).nanoseconds / 1e9
            safety_recent = elapsed < self.safety_timeout
        
        # Priority 1: Safety commands when safety is active
        if self.safety_active or safety_recent:
            output = self.safety_cmd
            source = "safety"
        else:
            # Priority 2: Navigation commands
            if self.last_nav_time is not None:
                elapsed = (now - self.last_nav_time).nanoseconds / 1e9
                if elapsed < self.nav_timeout:
                    output = self.nav_cmd
                    source = "nav"
                # else: timeout, output stays zero
        
        self.cmd_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    
    try:
        while rclpy.ok() and not node._shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
