#ifndef __RGBD_SLAM_NODE_HPP__
#define __RGBD_SLAM_NODE_HPP__

#include <memory>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "utility.hpp"

class RgbdSlamNode : public rclcpp::Node
{
public:
  using ImageMsg = sensor_msgs::msg::Image;

  explicit RgbdSlamNode(ORB_SLAM3::System * pSLAM);
  ~RgbdSlamNode();

private:
  // Callback signature MUST use ConstSharedPtr& for ROS2 message_filters
  void GrabRGBD(const ImageMsg::ConstSharedPtr & msgRGB,
                const ImageMsg::ConstSharedPtr & msgD);

  using ApproxSyncPolicy =
    message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

  ORB_SLAM3::System * m_SLAM;

  // Subscribers
  message_filters::Subscriber<ImageMsg> rgb_sub_;
  message_filters::Subscriber<ImageMsg> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> syncApproximate_;

  // Pose publisher: /orbslam3/pose
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

#endif  // __RGBD_SLAM_NODE_HPP__
