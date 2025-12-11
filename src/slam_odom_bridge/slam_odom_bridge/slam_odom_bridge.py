#!/usr/bin/env python3
import math
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    )

def quaternion_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)

class SlamOdomBridge(Node):

    def __init__(self):
        super().__init__('slam_odom_bridge')

        # ---- Parameters (change via ROS params if you want) ----
        self.declare_parameter('slam_pose_topic', '/orbslam3/pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_map_to_odom', True)

        slam_pose_topic = self.get_parameter(
            'slam_pose_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_link_frame = self.get_parameter(
            'base_link_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.publish_map_to_odom = self.get_parameter(
            'publish_map_to_odom').get_parameter_value().bool_value

        # Publishers / TF
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Quaternion for Optical (Right-Down-Forward) to Base (Forward-Left-Up)
        # R_optical_to_base:
        # X_base = Z_opt
        # Y_base = -X_opt
        # Z_base = -Y_opt
        # q = [0.5, -0.5, 0.5, -0.5]  (x, y, z, w) -- Standard conversion
        self.q_opt_to_base = (0.5, -0.5, 0.5, -0.5)

        # If you want odom==map (no wheel odom), publish a static identity map->odom
        if self.publish_map_to_odom and self.map_frame and self.odom_frame:
            st = TransformStamped()
            st.header.stamp = self.get_clock().now().to_msg()
            st.header.frame_id = self.map_frame
            st.child_frame_id = self.odom_frame
            st.transform.translation.x = 0.0
            st.transform.translation.y = 0.0
            st.transform.translation.z = 0.0
            st.transform.rotation.x = 0.0
            st.transform.rotation.y = 0.0
            st.transform.rotation.z = 0.0
            st.transform.rotation.w = 1.0
            self.static_broadcaster.sendTransform(st)
            self.get_logger().info(
                f'Publishing static identity TF {self.map_frame} -> {self.odom_frame}')

        # Subscribe to ORB-SLAM3 pose
        self.sub = self.create_subscription(
            PoseStamped, slam_pose_topic, self.pose_cb, 10)
        self.get_logger().info('slam_odom_bridge node started')

    def pose_cb(self, msg: PoseStamped):
        # Build nav_msgs/Odometry (using odom_frame as parent and base_link_frame as child)
        odom = Odometry()
        # FIX: Use original timestamp to match sensor data
        # current_time = self.get_clock().now().to_msg()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_link_frame

        # Position Correction
        # P_base = q_fix * P_opt * q_fix_inv
        
        x_old = msg.pose.position.x
        y_old = msg.pose.position.y
        z_old = msg.pose.position.z

        odom.pose.pose.position.x = z_old
        odom.pose.pose.position.y = -x_old
        odom.pose.pose.position.z = -y_old

        # Orientation Correction
        # We perform a similarity transform (change of basis): 
        # q_new = q_fix * q_slam * q_fix_inverse
        
        q_slam = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        q_fix = self.q_opt_to_base
        q_fix_inv = quaternion_conjugate(q_fix)
        
        # 1. q_temp = q_slam * q_fix_inv
        q_temp = quaternion_multiply(q_slam, q_fix_inv)
        
        # 2. q_final = q_fix * q_temp
        q_final = quaternion_multiply(q_fix, q_temp)

        odom.pose.pose.orientation.x = q_final[0]
        odom.pose.pose.orientation.y = q_final[1]
        odom.pose.pose.orientation.z = q_final[2]
        odom.pose.pose.orientation.w = q_final[3]

        # Simple covariance (diagonal) so Nav2 doesn’t reject it
        cov = [0.0] * 36
        cov[0] = 0.05      # x
        cov[7] = 0.05      # y
        cov[14] = 0.10     # z
        cov[21] = 0.10     # roll
        cov[28] = 0.10     # pitch
        cov[35] = 0.20     # yaw
        odom.pose.covariance = cov

        # (Twist unknown – leave zeros)
        self.odom_pub.publish(odom)

        # Also broadcast TF odom -> base_link
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_link_frame
            t.transform.translation.x = odom.pose.pose.position.x
            t.transform.translation.y = odom.pose.pose.position.y
            t.transform.translation.z = odom.pose.pose.position.z
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = SlamOdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
