#pragma once

#include <chrono>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace forklift_control {

class SafetyMonitor {
public:
  // If `check_nonzero_int` is true the monitor will subscribe to an Int32 topic and
  // consider the heartbeat 'safe' only when the message `data` field is non-zero.
  // If `joy_button_index >= 0` the monitor will subscribe to a `sensor_msgs::msg::Joy`
  // topic and consider the heartbeat 'safe' only when the given `buttons[joy_button_index]` is non-zero.
  // If `joy_button_index < 0` and `check_nonzero_int` is true the monitor will subscribe to an
  // `std_msgs::msg::Int32` topic and consider non-zero data as safe. Otherwise the default is
  // a `std_msgs::msg::Bool` topic where true means safe.
  SafetyMonitor(rclcpp::Node& node,
                const std::string& heartbeat_topic = "/safety/heartbeat",
                std::chrono::milliseconds heartbeat_timeout = std::chrono::milliseconds(500),
                bool check_nonzero_int = false,
                int joy_button_index = -1);

  bool is_safe() const;

private:
  void heartbeat_cb_(const std_msgs::msg::Bool::SharedPtr msg);
  void heartbeat_int_cb_(const std_msgs::msg::Int32::SharedPtr msg);
  void heartbeat_joy_cb_(const sensor_msgs::msg::Joy::SharedPtr msg);

  // Use a SubscriptionBase to be able to hold subscriptions for multiple message types
  rclcpp::SubscriptionBase::SharedPtr heartbeat_sub_;
  rclcpp::SubscriptionBase::SharedPtr joystick_sub_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  std::chrono::nanoseconds heartbeat_timeout_;

  mutable std::mutex mtx_;
  rclcpp::Time last_heartbeat_time_;
  bool heartbeat_safe_ = false;
  bool have_heartbeat_ = false;
  bool check_nonzero_int_ = false;
  int joy_button_index_ = -1;
  bool joystick_safe_ = false;
  bool have_joy_ = false;
};

}  // namespace forklift_control

