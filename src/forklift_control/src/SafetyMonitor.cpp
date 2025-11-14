#include <functional>

#include "forklift_control/SafetyMonitor.hpp"

namespace forklift_control {

SafetyMonitor::SafetyMonitor(rclcpp::Node& node,
                             const std::string& heartbeat_topic,
                             std::chrono::milliseconds heartbeat_timeout,
                             bool check_nonzero_int,
                             int joy_button_index)
: clock_(node.get_clock()),
  heartbeat_timeout_(std::chrono::duration_cast<std::chrono::nanoseconds>(heartbeat_timeout)),
  last_heartbeat_time_(0, 0, clock_->get_clock_type()),
  check_nonzero_int_(check_nonzero_int),
  joy_button_index_(joy_button_index),
  logger_(node.get_logger())
{
  // Subscribe to heartbeat (Bool or Int32)
  if (check_nonzero_int_) {
    // Subscribe to an Int32 heartbeat and treat non-zero as safe
    heartbeat_sub_ = node.create_subscription<std_msgs::msg::Int32>(
      heartbeat_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&SafetyMonitor::heartbeat_int_cb_, this, std::placeholders::_1));
  } else {
    // Default behaviour: Bool heartbeat
    heartbeat_sub_ = node.create_subscription<std_msgs::msg::Bool>(
      heartbeat_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&SafetyMonitor::heartbeat_cb_, this, std::placeholders::_1));
  }

  // If a joystick button index was provided, subscribe to /joy and use the button
  // state as an additional safety requirement.
  if (joy_button_index_ >= 0) {
    joystick_sub_ = node.create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      rclcpp::SensorDataQoS(),
      std::bind(&SafetyMonitor::heartbeat_joy_cb_, this, std::placeholders::_1));
  }
}

void SafetyMonitor::heartbeat_cb_(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_heartbeat_time_ = clock_->now();
  // For Bool messages we accept true/false directly
  heartbeat_safe_ = msg->data;
  have_heartbeat_ = true;
  RCLCPP_DEBUG(
    logger_,
    "Heartbeat Bool received: data=%d, heartbeat_safe=%d, time=%.3f",
    static_cast<int>(msg->data),
    static_cast<int>(heartbeat_safe_),
    last_heartbeat_time_.seconds());
}

void SafetyMonitor::heartbeat_int_cb_(const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_heartbeat_time_ = clock_->now();
  // Only set safe when the integer field is non-zero
  heartbeat_safe_ = (msg->data != 0);
  have_heartbeat_ = true;
  RCLCPP_DEBUG(
    logger_,
    "Heartbeat Int received: data=%d, heartbeat_safe=%d, time=%.3f",
    static_cast<int>(msg->data),
    static_cast<int>(heartbeat_safe_),
    last_heartbeat_time_.seconds());
}

void SafetyMonitor::heartbeat_joy_cb_(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);

  // If the buttons array doesn't contain the requested index, consider it not-safe
  if (joy_button_index_ < 0) {
    joystick_safe_ = false;
    have_joy_ = false;
    last_joy_button_state_ = false;
    return;
  }

  if (static_cast<size_t>(joy_button_index_) < msg->buttons.size()) {
    bool pressed = (msg->buttons[joy_button_index_] != 0);
    have_joy_ = true;

    // Rising edge: was not pressed before, now is pressed
    if (pressed && !last_joy_button_state_) {
      // Toggle safety latch
      joystick_safe_ = !joystick_safe_;
      RCLCPP_INFO(
        logger_,
        "Joy toggle: button[%d] pressed -> joystick_safe=%d",
        joy_button_index_,
        static_cast<int>(joystick_safe_));
    }

    // Remember last state so we only toggle on press, not while held
    last_joy_button_state_ = pressed;

    RCLCPP_DEBUG(
      logger_,
      "Joy received: button[%d]=%d => joystick_safe=%d (latched)",
      joy_button_index_,
      static_cast<int>(msg->buttons[joy_button_index_]),
      static_cast<int>(joystick_safe_));
  } else {
    joystick_safe_ = false;
    have_joy_ = true;  // we did receive a message but index out of range
    last_joy_button_state_ = false;
    RCLCPP_WARN(
      logger_,
      "Joy received but buttons array too small: size=%zu, requested index=%d",
      msg->buttons.size(),
      joy_button_index_);
  }
}

bool SafetyMonitor::is_safe() const
{
  const auto now = clock_->now();

  std::lock_guard<std::mutex> lk(mtx_);
  // Require a valid heartbeat
  if (!have_heartbeat_) {
    RCLCPP_DEBUG(logger_, "is_safe() -> false: no heartbeat received yet");
    return false;
  }

  const auto elapsed = now - last_heartbeat_time_;
  if (elapsed.nanoseconds() > heartbeat_timeout_.count()) {
    RCLCPP_DEBUG(
      logger_,
      "is_safe() -> false: heartbeat timeout (elapsed=%.1f ms)",
      elapsed.nanoseconds() / 1e6);
    return false;
  }

  // If joystick monitoring is enabled, require that it's been seen and is safe (latched)
  if (joy_button_index_ >= 0) {
    if (!have_joy_) {
      RCLCPP_DEBUG(
        logger_,
        "is_safe() -> false: no joystick message seen yet (waiting for button %d)",
        joy_button_index_);
      return false;
    }
    const bool ok = (heartbeat_safe_ && joystick_safe_);
    RCLCPP_DEBUG(
      logger_,
      "is_safe() -> %s (heartbeat_safe=%d, joystick_safe=%d)",
      ok ? "true" : "false",
      static_cast<int>(heartbeat_safe_),
      static_cast<int>(joystick_safe_));
    return ok;
  }

  // Default behaviour: only heartbeat required
  RCLCPP_DEBUG(
    logger_,
    "is_safe() -> %s (heartbeat_safe=%d)",
    heartbeat_safe_ ? "true" : "false",
    static_cast<int>(heartbeat_safe_));
  return heartbeat_safe_;
}

}  // namespace forklift_control
