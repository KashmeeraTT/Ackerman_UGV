#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class SlamOdomBridge(Node):
    """
    Subscribes to a PoseStamped from ORB-SLAM3 and publishes:
      - nav_msgs/Odometry on /odom
      - TF transform odom -> base_link (or camera_link)
    We treat the SLAM pose as 'visual odometry' in the odom frame.
    """

    def __init__(self):
        super().__init__('slam_odom_bridge')

        # Parameters so you can adapt to your topic & frames
        self.declare_parameter('slam_pose_topic', '/orbslam3/pose')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')  # use 'camera_link' if you prefer

        slam_pose_topic = self.get_parameter(
            'slam_pose_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter(
            'odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame').get_parameter_value().string_value

        self.get_logger().info(
            f"Subscribing to SLAM pose: {slam_pose_topic}\n"
            f"Publishing /odom with frame_id={self.odom_frame}, child_frame_id={self.base_frame}"
        )

        # TF broadcaster and odom publisher
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)

        # Subscribe to SLAM pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            slam_pose_topic,
            self.pose_callback,
            50
        )

    def pose_callback(self, msg: PoseStamped):
        # 1) Publish TF odom -> base_frame
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # 2) Publish nav_msgs/Odometry on /odom
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = msg.pose
        # Leave twist zero for now (SLAM doesn’t directly give velocity)
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SlamOdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
