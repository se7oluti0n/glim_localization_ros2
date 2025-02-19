#pragma once

#include <any>
#include <atomic>
#include <thread>
#include <chrono>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <std_srvs/srv/trigger.hpp>

#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>

namespace spdlog {
class logger;
}

namespace glim {

class TrajectoryManager;

/**
 * @brief Rviz-based viewer
 */
class RvizViewer : public ExtensionModuleROS2 {
public:
  RvizViewer();
  ~RvizViewer();

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;

private:
  void set_callbacks();
  void odometry_new_frame(const EstimationFrame::ConstPtr& new_frame);
  void globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps);
  void on_update_active_submaps(const std::map<int, SubMap::Ptr>& submaps);
  void on_localization_submap(const std::vector<SubMap::Ptr>& submaps);
  void on_submap_debug(gtsam_points::PointCloud::ConstPtr submap, const Eigen::Isometry3d& pose);
  void invoke(const std::function<void()>& task);

  void on_user_event(int pose_id);
  void on_user_load_map();

  void spin_once();

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  rclcpp::Time last_globalmap_pub_time;
  Eigen::Isometry3d T_imu_lidar;

  std::string imu_frame_id;
  std::string lidar_frame_id;
  std::string base_frame_id;
  std::string odom_frame_id;
  std::string map_frame_id;
  bool publish_imu2lidar;
  bool publish_active_submaps;
  bool publish_topic_odom;
  double tf_time_offset;
  double rviz_random_sampling_rate;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> points_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> aligned_points_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> map_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> submap_pub;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Header>> user_event_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Point>> point_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> submap_pose_pub;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr load_map_client;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;


  std::mutex trajectory_mutex;
  std::unique_ptr<TrajectoryManager> trajectory;

  std::vector<SubMap::Ptr> submaps;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
};
}  // namespace glim
