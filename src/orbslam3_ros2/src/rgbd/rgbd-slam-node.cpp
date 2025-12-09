#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Tracking.h"  // for ORB_SLAM3::Tracking::OK

using std::placeholders::_1;
using std::placeholders::_2;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System * pSLAM)
: Node("ORB_SLAM3_ROS2"), m_SLAM(pSLAM)
{
  // Subscribe to Orbbec camera topics
  rgb_sub_.subscribe(this, "/camera/color/image_raw");
  depth_sub_.subscribe(this, "/camera/depth/image_raw");

  // Approximate time synchronization
  syncApproximate_ =
    std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
      ApproxSyncPolicy(10), rgb_sub_, depth_sub_);

  // One functor, two placeholders – ROS2 message_filters style
  syncApproximate_->registerCallback(
    std::bind(&RgbdSlamNode::GrabRGBD, this, _1, _2));

  // Publish SLAM pose as geometry_msgs/PoseStamped
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "orbslam3/pose", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "RGBD SLAM node constructed");
}

RgbdSlamNode::~RgbdSlamNode()
{
  if (m_SLAM) {
    RCLCPP_INFO(this->get_logger(), "Shutting down ORB-SLAM3 and saving trajectory...");
    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  }
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::ConstSharedPtr & msgRGB,
                            const ImageMsg::ConstSharedPtr & msgD)
{
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  cv_bridge::CvImageConstPtr cv_ptrD;

  // Convert RGB image
  try {
    cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (RGB): %s", e.what());
    return;
  }

  // Convert depth image
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (Depth): %s", e.what());
    return;
  }

  // In your ORB-SLAM3 build, TrackRGBD returns Sophus::SE3f (Tcw)
  Sophus::SE3f Tcw = m_SLAM->TrackRGBD(
    cv_ptrRGB->image,
    cv_ptrD->image,
    Utility::StampToSec(msgRGB->header.stamp));

  // Only publish when tracking is OK
  if (m_SLAM->GetTrackingState() != ORB_SLAM3::Tracking::OK) {
    return;
  }

  // Convert to Twc (world pose of the camera)
  Sophus::SE3f Twc = Tcw.inverse();

  Eigen::Vector3f t = Twc.translation();
  Eigen::Quaternionf q(Twc.rotationMatrix());

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = msgRGB->header.stamp;
  pose.header.frame_id = "map";   // SLAM world frame

  // Position
  pose.pose.position.x = static_cast<double>(t.x());
  pose.pose.position.y = static_cast<double>(t.y());
  pose.pose.position.z = static_cast<double>(t.z());

  // Orientation
  pose.pose.orientation.x = static_cast<double>(q.x());
  pose.pose.orientation.y = static_cast<double>(q.y());
  pose.pose.orientation.z = static_cast<double>(q.z());
  pose.pose.orientation.w = static_cast<double>(q.w());

  pose_pub_->publish(pose);
}
