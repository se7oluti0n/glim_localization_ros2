#include <glim_ros/glim_localization.hpp>
#include <glim/mapping/localization.hpp>

namespace glim {

GlimLocalization::GlimLocalization(const rclcpp::NodeOptions& options) :
  GlimROS(options)
{
}

GlimLocalization::~GlimLocalization() {
  GlimROS::~GlimROS();
}

}