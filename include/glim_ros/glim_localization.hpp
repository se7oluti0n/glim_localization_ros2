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

namespace glim {

class Localization;

class GlimLocalization : public GlimROS {
public:
  GlimLocalization(const rclcpp::NodeOptions& options);
  ~GlimLocalization();

private:

  std::shared_ptr<glim::Localization> localization_module;
};

}  // namespace glim
