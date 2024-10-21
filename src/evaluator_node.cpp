#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace glim {

class EvaluatorNode : public rclcpp::Node {
public:
  EvaluatorNode(rclcpp::NodeOptions& options) : rclcpp::Node("evaluator_node", options) {
    bool debug = false;
    this->declare_parameter("debug", debug);
    this->get_parameter("debug", debug);


    using std::placeholders::_1;
    const std::string imu_topic = "glim_ros/user_event";
    const std::string points_topic = "glim_ros/pose";

    rclcpp::QoS map_qos(10);
    // imu_qos.get_rmw_qos_profile().depth = 1000;
    event_sub = this->create_subscription<std_msgs::msg::Header>(imu_topic, map_qos, 
    [this](const std_msgs::msg::Header::SharedPtr msg) {
      const double stamp = msg->stamp.sec + msg->stamp.nanosec / 1e9;
      const double pose_stamp = latest_pose.header.stamp.sec + latest_pose.header.stamp.nanosec / 1e9;
      // validator->imu_callback(stamp, {a.x, a.y, a.z}, {w.x, w.y, w.z});
       RCLCPP_INFO_STREAM(this->get_logger(), 
        "event stamp: " << stamp << ", pose stamp: " << pose_stamp << "\n pose id: "
         << msg->frame_id 
         << ", Pose " << this->latest_pose.pose.position.x << ", " << this->latest_pose.pose.position.y << ", " << this->latest_pose.pose.position.z
         << ", Quat: " << this->latest_pose.pose.orientation.x << ", " << this->latest_pose.pose.orientation.y
         << ", " << this->latest_pose.pose.orientation.z << ", " << this->latest_pose.pose.orientation.w);
    });

    pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(points_topic, map_qos, 
    [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      this->latest_pose = *msg;
    });


  }

private:

  geometry_msgs::msg::PoseStamped latest_pose;

  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr event_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
};

}  // namespace glim

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  rclcpp::spin(std::make_shared<glim::EvaluatorNode>(options));
  rclcpp::shutdown();

  return 0;
}