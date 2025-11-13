#pragma once

#include <chrono>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace forklift_control {

class SafetyMonitor {
public:
  SafetyMonitor(rclcpp::Node& node,
                const std::string& heartbeat_topic = "/safety/heartbeat",
                std::chrono::milliseconds heartbeat_timeout = std::chrono::milliseconds(500));

  bool is_safe() const;

private:
  void heartbeat_cb_(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_sub_;
  rclcpp::Clock::SharedPtr clock_;
  std::chrono::nanoseconds heartbeat_timeout_;

  mutable std::mutex mtx_;
  rclcpp::Time last_heartbeat_time_;
  bool heartbeat_safe_ = false;
  bool have_heartbeat_ = false;
};

}  // namespace forklift_control

