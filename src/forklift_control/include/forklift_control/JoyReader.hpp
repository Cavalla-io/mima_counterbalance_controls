#pragma once
#include <mutex>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace forklift_control {

struct JoyState {
  float lx=0.f, ly=0.f, rx=0.f, ry=0.f, lt=0.f, rt=0.f;
  int A=0, B=0, X=0, Y=0, LB=0, RB=0, SELECT=0, START=0;
};

class JoyReader {
public:
  // Attach to an existing node and subscribe
  JoyReader(rclcpp::Node& node,
            const std::string& topic = "/joy",
            const rclcpp::QoS& qos = rclcpp::SensorDataQoS());

  // Thread-safe copy of latest values
  JoyState latest() const;
  bool has_message() const;
  // Time of the last received message (clock used is the node's clock)
  rclcpp::Time last_msg_time() const;

private:
  void cb_(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  mutable std::mutex mtx_;
  JoyState state_;
  bool have_data_ = false;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time last_msg_time_;
};

} // namespace forklift_control
