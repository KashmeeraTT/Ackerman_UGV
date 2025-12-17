#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math
import time

class SmartJointPublisher(Node):
    def __init__(self):
        super().__init__('smart_joint_publisher')
        
        # Publishers and Subscribers
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz

        # Parameters
        self.declare_parameter('wheelbase', 0.60) # Dist between front/rear axles
        self.declare_parameter('track_width', 1.16) # Dist between left/right wheels
        self.declare_parameter('max_steer_angle', 0.5) # Radians
        
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.max_steer_angle = self.get_parameter('max_steer_angle').value

        # State variables
        self.current_speed = 0.0
        self.current_steering_angle = 0.0
        self.wheel_rotation = 0.0
        self.last_time = time.time()

        # Joint Names
        self.steering_joints = ['front_left_steering_joint', 'front_right_steering_joint']
        self.traction_joints = ['rear_left_wheel_joint', 'rear_right_wheel_joint', 
                                'front_left_wheel_joint', 'front_right_wheel_joint']
        
        self.get_logger().info('Smart Joint Publisher Started')

    def cmd_vel_callback(self, msg):
        # Estimate steering angle from angular z and linear x
        # Angular = (Linear / Wheelbase) * tan(SteerAngle)
        # tan(SteerAngle) = (Angular * Wheelbase) / Linear
        
        v = msg.linear.x
        w = msg.angular.z
        
        self.current_speed = v

        if abs(v) > 0.01:
            steer = math.atan( (w * self.wheelbase) / v )
        else:
            # If not moving linearly, we might be trying to turn in place (not possible for Ackermann)
            # but for visualization we can just show the wheel turn.
            # However, standard formula breaks. Let's just scale it.
            steer = w # simplistic approx for static steering check
            
        # Clamp steering
        self.current_steering_angle = max(-self.max_steer_angle, min(self.max_steer_angle, steer))

    def timer_callback(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Update wheel rotation for animation (visual only effect)
        # s = r * theta -> theta = s / r. Assume radius ~0.15
        move_dist = self.current_speed * dt
        wheel_rot_inc = move_dist / 0.15 
        self.wheel_rotation += wheel_rot_inc
        
        # Populate message
        msg.name = self.steering_joints + self.traction_joints
        
        # Ackermann geometry: inner wheel turns more than outer wheel.
        # But for simple viz, equal steering is often "good enough".
        # Let's try to be slightly smarter? No, keep it simple for now to ensure robustness.
        # Both front wheels steer same angle for this visualizer.
        
        steering_pos = [self.current_steering_angle, self.current_steering_angle]
        wheel_pos = [self.wheel_rotation, self.wheel_rotation, self.wheel_rotation, self.wheel_rotation]
        
        msg.position = steering_pos + wheel_pos
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SmartJointPublisher()
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

