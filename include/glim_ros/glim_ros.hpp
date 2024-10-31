#pragma once

#include <any>
#include <deque>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <Eigen/Geometry>

namespace glim {
class TimeKeeper;
class CloudPreprocessor;
class AsyncOdometryEstimation;
class AsyncSubMapping;
class AsyncGlobalMapping;

class ExtensionModule;
class GenericTopicSubscription;

class GlimROS : public rclcpp::Node {
public:
  GlimROS(const rclcpp::NodeOptions& options);
  ~GlimROS();

  bool needs_wait();
  void timer_callback();

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void raw_odom_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  size_t points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void wait(bool auto_quit = false);
  void save(const std::string& path);

  const std::vector<std::shared_ptr<GenericTopicSubscription>>& extension_subscriptions();
  void handle_save_map_sevice(const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response);
private:
  void setup_localization();
  void handle_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose);
  void handle_reloc(const geometry_msgs::msg::Point::ConstSharedPtr point);
  void handle_load_map_sevice(const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response);
protected:
  std::unique_ptr<glim::TimeKeeper> time_keeper;
  std::unique_ptr<glim::CloudPreprocessor> preprocessor;

  std::shared_ptr<glim::AsyncOdometryEstimation> odometry_estimation;
  std::unique_ptr<glim::AsyncSubMapping> sub_mapping;
  std::unique_ptr<glim::AsyncGlobalMapping> global_mapping;

  bool keep_raw_points;
  double imu_time_offset;
  double points_time_offset;
  double acc_scale;

  // Extension modulles
  std::vector<std::shared_ptr<ExtensionModule>> extension_modules;
  std::vector<std::shared_ptr<GenericTopicSubscription>> extension_subs;

  // ROS-related
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr raw_odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;
  image_transport::Subscriber image_sub;

  bool force_create_submap_flag = false;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv;

  // localization
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr reloc_point_sub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_srv;
  Eigen::Isometry3d initial_pose_;

  std::string map_path;

};

}  // namespace glim
