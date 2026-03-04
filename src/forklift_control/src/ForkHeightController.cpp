#include "forklift_control/ForkHeightController.hpp"
#include <algorithm>
#include <cmath>
#include <string>

namespace forklift_control {

static constexpr int LIFT_PWM_MAX    = 100;
static constexpr int LOWER_VALVE_MAX = 200;

ForkHeightController::ForkHeightController(
    rclcpp::Node& node,
    const std::string& cmd_topic,
    const std::string& pos_topic,
    std::chrono::milliseconds cmd_timeout)
  : clock_(node.get_clock()),
    logger_(node.get_logger()),
    cmd_timeout_(cmd_timeout),
    node_(node),
    last_cmd_time_(0, 0, clock_->get_clock_type())
{
  // Declare PID parameters
  if (!node_.has_parameter("fork_auto_kp"))
    node_.declare_parameter<double>("fork_auto_kp", 0.05);
  if (!node_.has_parameter("fork_auto_ki"))
    node_.declare_parameter<double>("fork_auto_ki", 0.0);
  if (!node_.has_parameter("fork_auto_kd"))
    node_.declare_parameter<double>("fork_auto_kd", 0.0);
  if (!node_.has_parameter("fork_auto_deadband_mm"))
    node_.declare_parameter<double>("fork_auto_deadband_mm", 15.0);
  if (!node_.has_parameter("fork_auto_integral_clamp"))
    node_.declare_parameter<double>("fork_auto_integral_clamp", 50.0);

  cmd_sub_ = node.create_subscription<std_msgs::msg::Float64>(
    cmd_topic, rclcpp::QoS(1),
    [this](const std_msgs::msg::Float64::SharedPtr msg) { cmd_cb_(msg); });

  pos_sub_ = node.create_subscription<std_msgs::msg::String>(
    pos_topic, rclcpp::QoS(1),
    [this](const std_msgs::msg::String::SharedPtr msg) { pos_cb_(msg); });
}

void ForkHeightController::cmd_cb_(const std_msgs::msg::Float64::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  target_m_ = msg->data;
  have_cmd_ = true;
  last_cmd_time_ = clock_->now();
}

void ForkHeightController::pos_cb_(const std_msgs::msg::String::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mtx_);

  // Parse JSON: {"height": <mm>, "status": <0-3>, "desc": "..."}
  // Simple string parsing to avoid nlohmann dependency
  const std::string& json = msg->data;

  auto extract_number = [&](const std::string& key) -> double {
    std::string search = "\"" + key + "\"";
    auto pos = json.find(search);
    if (pos == std::string::npos) return -1.0;
    pos = json.find(':', pos + search.size());
    if (pos == std::string::npos) return -1.0;
    pos++; // skip ':'
    // skip whitespace
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
    try {
      size_t end;
      double val = std::stod(json.substr(pos), &end);
      return val;
    } catch (...) {
      return -1.0;
    }
  };

  double h = extract_number("height");
  double s = extract_number("status");

  if (h >= 0.0) {
    pos_state_.height_mm = h;
    pos_state_.status = static_cast<int>(s);
    pos_state_.valid = true;
  }
}

bool ForkHeightController::is_auto_mode() const {
  std::lock_guard<std::mutex> lk(mtx_);

  if (!have_cmd_) return false;
  if (target_m_ < 0.0) return false;
  if (target_m_ > 3.0) return false;

  // Check timeout
  const auto now = clock_->now();
  const auto elapsed = now - last_cmd_time_;
  if (elapsed.nanoseconds() > cmd_timeout_.count() * 1000000LL) return false;

  return true;
}

ForkAutoCommand ForkHeightController::compute(double dt_s) {
  std::lock_guard<std::mutex> lk(mtx_);
  ForkAutoCommand cmd;

  // No position data -> stop
  if (!pos_state_.valid) {
    reset_pid();
    return cmd;  // STOP, 0
  }

  // Safety violation -> stop
  if (pos_state_.status == 3) {
    reset_pid();
    return cmd;  // STOP, 0
  }

  const double target_mm = target_m_ * 1000.0;
  const double current_mm = pos_state_.height_mm;
  const double error = target_mm - current_mm;

  const double kp = node_.get_parameter("fork_auto_kp").as_double();
  const double ki = node_.get_parameter("fork_auto_ki").as_double();
  const double kd = node_.get_parameter("fork_auto_kd").as_double();
  const double deadband_mm = node_.get_parameter("fork_auto_deadband_mm").as_double();
  const double integral_clamp = node_.get_parameter("fork_auto_integral_clamp").as_double();

  // Deadband check
  if (std::fabs(error) <= deadband_mm) {
    reset_pid();
    return cmd;  // STOP (at target)
  }

  // PID computation
  integral_ += error * dt_s;
  integral_ = std::clamp(integral_, -integral_clamp, integral_clamp);

  double derivative = (dt_s > 0.0) ? (error - prev_error_) / dt_s : 0.0;
  prev_error_ = error;

  double output = kp * error + ki * integral_ + kd * derivative;

  if (error > 0.0) {
    // Need to LIFT
    cmd.action = ForkAction::LIFT;
    int pwm = static_cast<int>(std::round(std::fabs(output)));
    cmd.value = std::clamp(pwm, 0, LIFT_PWM_MAX);
  } else {
    // Need to LOWER - scale x2 per plan
    cmd.action = ForkAction::LOWER;
    int valve = static_cast<int>(std::round(std::fabs(output) * 2.0));
    cmd.value = std::clamp(valve, 0, LOWER_VALVE_MAX);
  }

  return cmd;
}

void ForkHeightController::reset_pid() {
  // Note: caller must hold mtx_ or this is called from compute() which holds it
  integral_ = 0.0;
  prev_error_ = 0.0;
}

bool ForkHeightController::position_safety_violation() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return pos_state_.valid && pos_state_.status == 3;
}

double ForkHeightController::target_height_mm() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return target_m_ * 1000.0;
}

double ForkHeightController::current_height_mm() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return pos_state_.height_mm;
}

}  // namespace forklift_control
