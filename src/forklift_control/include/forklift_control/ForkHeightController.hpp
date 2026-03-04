#pragma once

#include <chrono>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

namespace forklift_control {

struct ForkPositionState {
  double height_mm = 0.0;
  int    status    = -1;   // -1 = no data, 0-3 from ForkPublisher
  bool   valid     = false;
};

enum class ForkAction { STOP, LIFT, LOWER };

struct ForkAutoCommand {
  ForkAction action = ForkAction::STOP;
  int        value  = 0;   // PWM (0-100) for LIFT, valve (0-200) for LOWER
};

class ForkHeightController {
public:
  ForkHeightController(rclcpp::Node& node,
                       const std::string& cmd_topic = "/fork_height_cmd",
                       const std::string& pos_topic = "/fork_position",
                       std::chrono::milliseconds cmd_timeout = std::chrono::milliseconds(500));

  bool is_auto_mode() const;
  ForkAutoCommand compute(double dt_s);
  void reset_pid();
  bool position_safety_violation() const;

  // For debug logging
  double target_height_mm() const;
  double current_height_mm() const;

private:
  void cmd_cb_(const std_msgs::msg::Float64::SharedPtr msg);
  void pos_cb_(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  pos_sub_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  std::chrono::nanoseconds cmd_timeout_;

  // ROS2 parameters (declared on parent node)
  rclcpp::Node& node_;

  mutable std::mutex mtx_;

  // Command state
  double  target_m_       = -1.0;
  bool    have_cmd_       = false;
  rclcpp::Time last_cmd_time_;

  // Position state
  ForkPositionState pos_state_;

  // PID state
  double integral_   = 0.0;
  double prev_error_ = 0.0;
};

}  // namespace forklift_control
