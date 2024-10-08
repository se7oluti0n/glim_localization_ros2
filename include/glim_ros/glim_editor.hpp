#pragma once

#include <any>
#include <deque>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace glim {

class ExtensionModule;
class GenericTopicSubscription;

  class SubmapPublisher: public rclcpp::Node {
  public:
    SubmapPublisher(const rclcpp::NodeOptions& options);

    ~SubmapPublisher();
    void timer_callback();

    const std::vector<std::shared_ptr<GenericTopicSubscription>>& extension_subscriptions();

  private:
    // Extension modulles
    std::vector<std::shared_ptr<ExtensionModule>> extension_modules;
    std::vector<std::shared_ptr<GenericTopicSubscription>> extension_subs;

    rclcpp::TimerBase::SharedPtr timer;

};
}