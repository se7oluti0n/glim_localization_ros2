#include <glim_ros/glim_localization.hpp>
#include <glim/mapping/localization.hpp>

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/odometry/async_odometry_estimation.hpp>
#include <glim/mapping/async_sub_mapping.hpp>
#include <glim/mapping/async_global_mapping.hpp>
#include <glim_ros/ros_compatibility.hpp>

namespace glim {

using std::placeholders::_1;

GlimLocalization::GlimLocalization(const rclcpp::NodeOptions& options) :
  GlimROS(options)
{

  initial_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10, std::bind(&GlimLocalization::handle_initial_pose, this, _1));

}

GlimLocalization::~GlimLocalization() {
  GlimROS::~GlimROS();
}

void GlimLocalization::handle_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  Eigen::Quaterniond quat(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
  );

  Eigen::Vector3d translation(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z
  );

  initial_pose_.translation() = translation;
  initial_pose_.linear() = quat.toRotationMatrix();

  auto most_recent_submap = sub_mapping->force_create_submap();

  global_mapping->relocalize(most_recent_submap, initial_pose_);

}

}