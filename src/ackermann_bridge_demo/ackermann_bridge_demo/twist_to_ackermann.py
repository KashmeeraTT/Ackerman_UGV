import math

from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class TwistToAck(Node):

    def __init__(self):
        super().__init__('ackermann_bridge_demo')
        # gp = self.get_parameter  # unused
        dp = self.declare_parameter
        self.L = dp('wheelbase_m', 1.26).get_parameter_value().double_value
        self.delta_max = math.radians(dp('max_steer_deg', 20.0).get_parameter_value().double_value)
        self.kappa_max = math.tan(self.delta_max) / self.L
        self.max_speed = dp('max_speed_mps', 0.5).get_parameter_value().double_value
        self.max_accel = dp('max_accel_mps2', 0.6).get_parameter_value().double_value
        self.cmd_timeout = dp('cmd_timeout_sec', 0.2).get_parameter_value().double_value
        self.v_eps = dp('v_epsilon_mps', 0.05).get_parameter_value().double_value
        self.allow_reverse = dp('allow_reverse', True).get_parameter_value().bool_value
        self.in_topic = dp('input_topic', '/cmd_vel').get_parameter_value().string_value
        self.out_topic = dp('output_topic', '/ackermann_cmd').get_parameter_value().string_value
        self.frame_id = dp('frame_id', 'base_link').get_parameter_value().string_value
        self.prev_speed = 0.0
        self.sub = self.create_subscription(Twist, self.in_topic, self.cb, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, self.out_topic, 10)
        self.timer = self.create_timer(0.05, self.watchdog)
        self.last_msg_time = None
        self.get_logger().info(f'L={self.L:.2f}m, kappa_max={self.kappa_max:.4f} 1/m')

    def cb(self, msg: Twist):
        now = self.get_clock().now()
        self.last_msg_time = now
        v, w = msg.linear.x, msg.angular.z
        if not self.allow_reverse and v < 0.0:
            v, w = 0.0, 0.0
        v_eff = v if abs(v) >= self.v_eps else (self.v_eps if v >= 0 else -self.v_eps)
        kappa = clamp(w / v_eff, -self.kappa_max, self.kappa_max)
        delta = clamp(math.atan(self.L * kappa), -self.delta_max, self.delta_max)
        # simple accel limit
        dv_max = self.max_accel * 0.05
        v_cmd = clamp(v, self.prev_speed - dv_max, self.prev_speed + dv_max)
        v_cmd = clamp(v_cmd, -self.max_speed, self.max_speed)
        self.prev_speed = v_cmd
        out = AckermannDriveStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self.frame_id
        out.drive = AckermannDrive()
        out.drive.speed = v_cmd
        out.drive.steering_angle = delta
        self.pub.publish(out)

    def watchdog(self):
        if self.last_msg_time is None:
            return
        if (self.get_clock().now() - self.last_msg_time).nanoseconds > int(self.cmd_timeout * 1e9):
            if abs(self.prev_speed) > 1e-3:
                self.get_logger().warn('cmd_vel timeout -> STOP')
            self.prev_speed = 0.0
            stop = AckermannDriveStamped()
            stop.header.stamp = self.get_clock().now().to_msg()
            stop.header.frame_id = self.frame_id
            stop.drive = AckermannDrive()
            self.pub.publish(stop)


def main():
    rclpy.init()
    rclpy.spin(TwistToAck())
    rclpy.shutdown()
