"""
Twist-to-Twist bridge with steering feedback for Ackermann UGV.

Subscribes to Nav2 cmd_vel, applies steering feedback speed limiting,
and outputs modified Twist to ESP32.
"""
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class SteeringFeedbackBridge(Node):

    def __init__(self):
        super().__init__('steering_feedback_bridge')
        dp = self.declare_parameter
        
        # Robot parameters
        self.L = dp('wheelbase_m', 0.60).get_parameter_value().double_value
        self.delta_max = math.radians(dp('max_steer_deg', 8.0).get_parameter_value().double_value)
        self.kappa_max = math.tan(self.delta_max) / self.L
        self.max_speed = dp('max_speed_mps', 0.5).get_parameter_value().double_value
        self.max_accel = dp('max_accel_mps2', 0.6).get_parameter_value().double_value
        self.cmd_timeout = dp('cmd_timeout_sec', 0.2).get_parameter_value().double_value
        self.v_eps = dp('v_epsilon_mps', 0.05).get_parameter_value().double_value
        self.allow_reverse = dp('allow_reverse', True).get_parameter_value().bool_value
        
        # Topics
        self.in_topic = dp('input_topic', '/cmd_vel_nav').get_parameter_value().string_value
        self.out_topic = dp('output_topic', '/cmd_vel_nav_mux').get_parameter_value().string_value
        
        # Steering feedback parameters
        self.use_steering_feedback = dp('use_steering_feedback', True).get_parameter_value().bool_value
        self.steer_error_speed_limit = dp('steer_error_speed_limit_deg', 5.0).get_parameter_value().double_value
        self.current_steering_angle = 0.0  # Actual steering angle from ESP32
        self.target_steering_angle = 0.0   # Calculated target
        
        self.prev_speed = 0.0
        
        # Subscriber and Publisher
        self.sub = self.create_subscription(Twist, self.in_topic, self.cmd_vel_cb, 10)
        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        
        # Steering angle feedback from ESP32
        self.steering_sub = self.create_subscription(
            Float32, '/ugv/steering_angle', self.steering_feedback_cb, 10)
        
        self.timer = self.create_timer(0.05, self.watchdog)
        self.last_msg_time = None
        
        self.get_logger().info(f'Wheelbase={self.L:.2f}m, max_steer={math.degrees(self.delta_max):.1f}deg')
        self.get_logger().info(f'Steering feedback: {"enabled" if self.use_steering_feedback else "disabled"}')
        self.get_logger().info(f'{self.in_topic} -> {self.out_topic}')

    def steering_feedback_cb(self, msg):
        """Callback for steering angle feedback from ESP32."""
        self.current_steering_angle = msg.data  # Degrees

    def cmd_vel_cb(self, msg: Twist):
        now = self.get_clock().now()
        self.last_msg_time = now
        v, w = msg.linear.x, msg.angular.z
        
        if not self.allow_reverse and v < 0.0:
            v, w = 0.0, 0.0
        
        # Calculate target steering angle for feedback comparison
        v_eff = v if abs(v) >= self.v_eps else (self.v_eps if v >= 0 else -self.v_eps)
        kappa = clamp(w / v_eff, -self.kappa_max, self.kappa_max)
        delta = clamp(math.atan(self.L * kappa), -self.delta_max, self.delta_max)
        self.target_steering_angle = math.degrees(delta)
        
        # Acceleration limiting
        dv_max = self.max_accel * 0.05
        v_cmd = clamp(v, self.prev_speed - dv_max, self.prev_speed + dv_max)
        v_cmd = clamp(v_cmd, -self.max_speed, self.max_speed)
        
        # Steering feedback: limit speed when steering is catching up
        if self.use_steering_feedback:
            steer_error = abs(self.target_steering_angle - self.current_steering_angle)
            if steer_error > self.steer_error_speed_limit:
                speed_factor = max(0.3, 1.0 - (steer_error - self.steer_error_speed_limit) / 15.0)
                v_cmd = v_cmd * speed_factor
        
        self.prev_speed = v_cmd
        
        # Output modified Twist
        out = Twist()
        out.linear.x = v_cmd
        out.angular.z = w  # Pass through angular velocity - ESP32 converts to steering
        self.pub.publish(out)

    def watchdog(self):
        if self.last_msg_time is None:
            return
        if (self.get_clock().now() - self.last_msg_time).nanoseconds > int(self.cmd_timeout * 1e9):
            if abs(self.prev_speed) > 1e-3:
                self.get_logger().warn('cmd_vel timeout -> STOP')
            self.prev_speed = 0.0
            stop = Twist()
            self.pub.publish(stop)


def main():
    rclpy.init()
    node = SteeringFeedbackBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
