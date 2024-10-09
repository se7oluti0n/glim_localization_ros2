#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <glim/util/config.hpp>
#include <glim/viewer/offline_viewer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <glim_ros/glim_editor.hpp>



int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto glim = std::make_shared<glim::SubmapPublisher>(options);

  // std::string config_path = "config";
  // if (argc > 1) {
  //    std::cout << "config_path command : " <<  argv[1] << std::endl;
  //    config_path = argv[1];
  // }

  // if (config_path[0] != '/') {
  //   // config_path is relative to the glim directory
  //   config_path = ament_index_cpp::get_package_share_directory("glim") + "/" + config_path;
  // }

  // std::cout << "config_path: " << config_path << std::endl;
  // glim::GlobalConfig::instance(config_path);

  std::string init_map_path;
  // if (argc >= 3) {
  //   init_map_path = argv[2];
  //   std::cout << "map_path=" << init_map_path << std::endl;
  // }

  glim::OfflineViewer viewer(init_map_path);

  std::thread viewer_thread([&](){
    viewer.wait();
  });



  rclcpp::spin(glim);
  rclcpp::shutdown();

  viewer.stop();
  viewer_thread.join();
}