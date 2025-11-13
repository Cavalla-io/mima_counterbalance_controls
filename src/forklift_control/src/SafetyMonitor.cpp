#include <functional>

#include "forklift_control/SafetyMonitor.hpp"

namespace forklift_control {

SafetyMonitor::SafetyMonitor(rclcpp::Node& node,
                             const std::string& heartbeat_topic,
                             std::chrono::milliseconds heartbeat_timeout)
: clock_(node.get_clock()),
  heartbeat_timeout_(std::chrono::duration_cast<std::chrono::nanoseconds>(heartbeat_timeout)),
  last_heartbeat_time_(0, 0, clock_->get_clock_type())
{
  heartbeat_sub_ = node.create_subscription<std_msgs::msg::Bool>(
    heartbeat_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&SafetyMonitor::heartbeat_cb_, this, std::placeholders::_1));
}

void SafetyMonitor::heartbeat_cb_(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_heartbeat_time_ = clock_->now();
  heartbeat_safe_ = msg->data;
  have_heartbeat_ = true;
}

bool SafetyMonitor::is_safe() const
{
  const auto now = clock_->now();

  std::lock_guard<std::mutex> lk(mtx_);
  if (!have_heartbeat_) return false;

  const auto elapsed = now - last_heartbeat_time_;
  if (elapsed.nanoseconds() > heartbeat_timeout_.count()) return false;

  return heartbeat_safe_;
}

}  // namespace forklift_control

