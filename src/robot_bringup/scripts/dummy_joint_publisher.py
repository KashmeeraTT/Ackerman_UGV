#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class DummyJointPublisher(Node):
    def __init__(self):
        super().__init__('dummy_joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        # Declare parameter for valid joints
        self.declare_parameter('joints', [
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
            'front_left_steering_joint',
            'front_left_wheel_joint',
            'front_right_steering_joint',
            'front_right_wheel_joint'
        ])
        
        self.joints = self.get_parameter('joints').value
        self.get_logger().info(f'Publishing 0.0 state for joints: {self.joints}')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joints
        msg.position = [0.0] * len(self.joints)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
