#include <glim_ros/glim_editor.hpp>

#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
namespace glim
{
      SubmapPublisher::SubmapPublisher(const rclcpp::NodeOptions& options):  Node("glim_ros", options){
      std::string config_path;
      this->declare_parameter<std::string>("config_path", "config");
      this->get_parameter<std::string>("config_path", config_path);

      if (config_path[0] != '/') {
        // config_path is relative to the glim directory
        config_path = ament_index_cpp::get_package_share_directory("glim") + "/" + config_path;
      }

      spdlog::info("config_path: {}", config_path);
      glim::GlobalConfig::instance(config_path);
      glim::Config config_editor(glim::GlobalConfig::get_config_path("config_editor"));

        // Extention modules
      const auto extensions = config_editor.param<std::vector<std::string>>("editor", "extension_modules");
      if (extensions && !extensions->empty()) {
        for (const auto& extension : *extensions) {
          if (extension.find("viewer") == std::string::npos && extension.find("monitor") == std::string::npos) {
            spdlog::warn("Extension modules are enabled!!");
            spdlog::warn("You must carefully check and follow the licenses of ext modules");

            try {
              const std::string config_ext_path = ament_index_cpp::get_package_share_directory("glim_ext") + "/config";
              spdlog::info("config_ext_path: {}", config_ext_path);
              glim::GlobalConfig::instance()->override_param<std::string>("global", "config_ext", config_ext_path);
            } catch (ament_index_cpp::PackageNotFoundError& e) {
              spdlog::warn("glim_ext package path was not found!!");
            }

            break;
          }
        }

        for (const auto& extension : *extensions) {
          spdlog::info("load {}", extension);
          auto ext_module = ExtensionModule::load_module(extension);
          if (ext_module == nullptr) {
            spdlog::error("failed to load {}", extension);
            continue;
          } else {
            extension_modules.push_back(ext_module);

            auto ext_module_ros = std::dynamic_pointer_cast<ExtensionModuleROS2>(ext_module);
            if (ext_module_ros) {
              const auto subs = ext_module_ros->create_subscriptions(*this);
              extension_subs.insert(extension_subs.end(), subs.begin(), subs.end());
            }
          }
        }
      }

      for (const auto& sub : this->extension_subscriptions()) {
        spdlog::debug("subscribe to {}", sub->topic);
        sub->create_subscriber(*this);
      }

      // Start timer
      timer = this->create_wall_timer(std::chrono::milliseconds(1), [this]() { timer_callback(); });

    }
    SubmapPublisher::~SubmapPublisher() {}

    const std::vector<std::shared_ptr<GenericTopicSubscription>>& SubmapPublisher::extension_subscriptions() {
      return extension_subs;
    }

    void SubmapPublisher::timer_callback() {
      for (const auto& ext_module : extension_modules) {
        if (!ext_module->ok()) {
          rclcpp::shutdown();
        }
      }
    }
} // namespace name

// RCLCPP_COMPONENTS_REGISTER_NODE(glim::SubmapPublisher);