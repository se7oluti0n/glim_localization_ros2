#pragma once

#include <any>
#include <deque>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "glim_ros.hpp"

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <Eigen/Geometry>

namespace glim {

class Localization;

class GlimLocalization : public GlimROS {
public:
  GlimLocalization(const rclcpp::NodeOptions& options);
  ~GlimLocalization();

private:
  void handle_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose);
private:

  std::shared_ptr<glim::Localization> localization_module;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub;
  Eigen::Isometry3d initial_pose_;

};

}  // namespace glim
