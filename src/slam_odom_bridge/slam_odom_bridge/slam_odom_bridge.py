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

def quaternion_normalize(q):
    """Normalize a quaternion to unit length."""
    x, y, z, w = q
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-10:
        return (0.0, 0.0, 0.0, 1.0)  # Return identity if degenerate
    return (x/norm, y/norm, z/norm, w/norm)

def is_valid_pose(position, orientation):
    """Check if pose contains valid (non-NaN, non-Inf) values."""
    values = [
        position.x, position.y, position.z,
        orientation.x, orientation.y, orientation.z, orientation.w
    ]
    for v in values:
        if math.isnan(v) or math.isinf(v):
            return False
    return True

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
        self.declare_parameter('publish_rate', 20.0)  # Hz for TF publishing
        self.declare_parameter('tracking_timeout', 1.0)  # seconds before warning about lost tracking

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
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.tracking_timeout = self.get_parameter(
            'tracking_timeout').get_parameter_value().double_value

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

        # State tracking for continuous TF publishing
        self.last_odom = None  # Store last computed odometry
        self.last_slam_time = None  # Time of last SLAM pose
        self.tracking_active = False  # Whether VSLAM is currently tracking
        self.tracking_lost_warned = False  # Avoid spamming warnings

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

        # Timer for continuous TF publishing (critical for Nav2)
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_cb)
        
        self.get_logger().info(
            f'slam_odom_bridge node started (TF publish rate: {publish_rate}Hz)')
        self.get_logger().info(
            'Publishing identity TF until VSLAM starts tracking...')

    def timer_cb(self):
        """Timer callback to continuously publish TF (even before VSLAM tracks)."""
        current_time = self.get_clock().now()
        
        # Check tracking status
        if self.last_slam_time is not None:
            elapsed = (current_time - self.last_slam_time).nanoseconds / 1e9
            if elapsed > self.tracking_timeout:
                if self.tracking_active:
                    self.tracking_active = False
                    if not self.tracking_lost_warned:
                        self.get_logger().warn(
                            f'VSLAM tracking lost (no pose for {elapsed:.1f}s), '
                            'using last known pose')
                        self.tracking_lost_warned = True
            else:
                if not self.tracking_active:
                    self.tracking_active = True
                    self.tracking_lost_warned = False
                    self.get_logger().info('VSLAM tracking recovered')
        
        if not self.publish_tf:
            return
            
        # Publish TF: use last known pose or identity
        if self.last_odom is not None:
            # Re-publish last known odometry with current timestamp
            self._publish_tf_from_odom(self.last_odom, current_time.to_msg())
        else:
            # Publish identity transform before VSLAM starts
            self._publish_identity_tf(current_time.to_msg())

    def _publish_identity_tf(self, stamp):
        """Publish identity transform for odom->base_link."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_link_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _publish_tf_from_odom(self, odom, stamp):
        """Publish TF from odometry message."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_link_frame
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def pose_cb(self, msg: PoseStamped):
        """Process SLAM pose and convert to odometry."""
        # Validate input pose (reject NaN/Inf values)
        if not is_valid_pose(msg.pose.position, msg.pose.orientation):
            self.get_logger().warn('Received invalid SLAM pose (NaN/Inf detected), ignoring')
            return
        
        # Update last SLAM time for tracking status
        self.last_slam_time = self.get_clock().now()
        
        # Build nav_msgs/Odometry (using odom_frame as parent and base_link_frame as child)
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_link_frame

        # Position Correction: Optical (RDF) to Base (FLU) frame
        # X_base = Z_opt, Y_base = -X_opt, Z_base = -Y_opt
        x_old = msg.pose.position.x
        y_old = msg.pose.position.y
        z_old = msg.pose.position.z

        odom.pose.pose.position.x = z_old
        odom.pose.pose.position.y = -x_old
        odom.pose.pose.position.z = -y_old

        # Orientation Correction: similarity transform (change of basis)
        # q_new = q_fix * q_slam * q_fix_inverse
        q_slam = quaternion_normalize((msg.pose.orientation.x, msg.pose.orientation.y, 
                                       msg.pose.orientation.z, msg.pose.orientation.w))
        q_fix = self.q_opt_to_base
        q_fix_inv = quaternion_conjugate(q_fix)
        
        q_temp = quaternion_multiply(q_slam, q_fix_inv)
        q_final = quaternion_normalize(quaternion_multiply(q_fix, q_temp))

        odom.pose.pose.orientation.x = q_final[0]
        odom.pose.pose.orientation.y = q_final[1]
        odom.pose.pose.orientation.z = q_final[2]
        odom.pose.pose.orientation.w = q_final[3]

        # Simple covariance (diagonal) so Nav2 doesn't reject it
        cov = [0.0] * 36
        cov[0] = 0.05      # x
        cov[7] = 0.05      # y
        cov[14] = 0.10     # z
        cov[21] = 0.10     # roll
        cov[28] = 0.10     # pitch
        cov[35] = 0.20     # yaw
        odom.pose.covariance = cov

        # Publish odometry
        self.odom_pub.publish(odom)
        
        # Store for timer-based TF republishing
        self.last_odom = odom


def main():
    rclpy.init()
    node = SlamOdomBridge()
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


