#include "rclcpp/rclcpp.hpp"
#include "forklift_control/JoyReader.hpp"
#include "forklift_control/DriveLogic.hpp"
#include "forklift_control/ForkLogic.hpp"
#include "forklift_control/SocketCanIface.hpp"
#include "forklift_control/SafetyMonitor.hpp"
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

// ---- Tunables (match teleop_ps.py) ----
static constexpr float MAX_SPEED_RPM   = 1200.0f;
static constexpr float MAX_STEER_DEG   = 90.0f;
static constexpr float DEADZONE        = 0.08f;
static constexpr float FORK_DEADBAND   = 0.05f;
static constexpr int   LIFT_PWM_MAX    = 100;   // 0..100
static constexpr int   LOWER_VALVE_MAX = 200;   // 0..200

// Deadzone + rescale [-1..1]
static inline float dz(float x, float d = DEADZONE) {
  float ax = std::fabs(x);
  if (ax < d) return 0.f;
  float s = (ax - d) / (1.f - d);
  s = std::min(1.f, std::max(0.f, s));
  return std::copysign(s, x);
}

static const char* safety_status_str(int code) {
  switch (code) {
    case 0: return "SAFE";
    case 1: return "TELEOP_UNFOCUSED";
    case 2: return "HIGH_LATENCY";
    case 3: return "CONTROLLER_DISCONNECTED";
    default: return "UNKNOWN";
  }
}

class ForkliftControlNode : public rclcpp::Node {
public:
  ForkliftControlNode()
  : rclcpp::Node("forklift_control"),
    joy_(*this, "/joy"),
    can_(declare_parameter<std::string>("can_iface", "can0")),
    drive_(can_, /*node_id=*/declare_parameter<int>("node_id", 3)),
  fork_(can_, drive_, /*node_id*/std::nullopt),
  safety_(*this, "/safety", 500ms, true, 4)
  {
    // Initial drive state
    drive_.set_ramps(1.0f, 1.0f);           // accel/decel seconds
    drive_.set_steering_deg(0.0f);
    drive_.set_direction_lowbits(false, false);
    drive_.set_speed_rpm(0.0f);
    drive_.set_descent_valve(0);
    drive_.set_lift_solenoid(false);
    drive_.set_lower_solenoid(false);

    // Initial pump ramps
    fork_.set_pump_ramps(1.0f, 1.0f);

    // Control @ 50 Hz
    ctrl_timer_ = create_wall_timer(20ms, [this]{ this->tick_(); });
    // Keepalives: resend last 0x200 and 0x300 frames periodically
    ka_drive_timer_ = create_wall_timer(200ms, [this]{ drive_.send_now(); });
    ka_fork_timer_  = create_wall_timer(200ms, [this]{ fork_.keepalive(); });

    const auto iface  = get_parameter("can_iface").as_string();
    const int  nodeid = static_cast<int>(get_parameter("node_id").as_int());
    RCLCPP_INFO(
      get_logger(),
      "ForkliftControlNode up; iface=%s node_id=%d (drive COB=0x%03X, fork COB=0x%03X); reading /joy",
      iface.c_str(),
      nodeid,
      static_cast<unsigned>(0x200 + nodeid),
      static_cast<unsigned>(0x300 + nodeid)
    );

    RCLCPP_WARN(
      get_logger(),
      "Safety heartbeat required; forklift remains UNSAFE until heartbeat reports safe.");

    // Parameter: how recent a /joy message must be (ms) to allow one-shot
    // execution when the safety heartbeat is missing.
    (void)declare_parameter<int>("recent_joy_window_ms", 200);
  }

private:
  void tick_() {
    const bool safety_ok = safety_.is_safe();
    const int safety_status = safety_.last_status_code();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "SafetyMonitor status=%d (%s) => safe=%d",
      safety_status,
      safety_status_str(safety_status),
      static_cast<int>(safety_ok));
    drive_.set_safe_state(safety_ok);
    fork_.set_safe_state(safety_ok);

    if (!joy_.has_message()) {
      apply_safe_outputs_();
      return;
    }

    const auto s = joy_.latest();

    if (!first_joy_received_) {
      first_joy_received_ = true;
      startup_lockout_active_ = !inputs_neutral_(s);
      if (startup_lockout_active_) {
        RCLCPP_WARN(
          get_logger(),
          "Joystick startup lockout: release all buttons, triggers, and sticks to enable control.");
      }
    }

    if (startup_lockout_active_) {
      if (!inputs_neutral_(s)) {
        apply_safe_outputs_();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "Joystick startup lockout active: release all controls to continue.");
        return;
      }
      startup_lockout_active_ = false;
      RCLCPP_INFO(get_logger(), "Joystick lockout cleared; controls enabled.");
    }

      // Allow a one-shot joystick command only if the last /joy message is
      // very recent (configured by 'recent_joy_window_ms'). Otherwise send
      // safe outputs and skip processing.
      bool recent_ok = false;
      if (joy_.has_message()) {
        const auto last = joy_.last_msg_time();
        const auto now = this->get_clock()->now();
        const auto elapsed_ns = (now - last).nanoseconds();
        const int ms = static_cast<int>(elapsed_ns / 1000000);
        const int recent_ms = static_cast<int>(get_parameter("recent_joy_window_ms").as_int());
        if (ms >= 0 && ms <= recent_ms) recent_ok = true;
      }
      if (!recent_ok) {
        apply_safe_outputs_();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "Not Seeing Recent Messages");
        return;
      }
      // otherwise fall through and execute the joystick-derived commands once

    // --- E-STOP toggle like python (X sets, Y clears) ---
    if (s.X)      estop_ = true;
    else if (s.Y) estop_ = false;

    // ----- Drive -----
    const float steer_cmd_deg = dz(s.lx) * MAX_STEER_DEG;

    // Raw trigger values (assumed already scaled 0..1 increasing with press)
    const float lt_n = s.lt;  // reverse
    const float rt_n = s.rt;  // forward

    // Direction bits (EPS, RT wins ties)
    constexpr float EPS = 0.05f;
    const bool rt_active = (rt_n > EPS && rt_n >= lt_n);
    const bool lt_active = (lt_n > EPS && lt_n >  rt_n);
    drive_.set_direction_lowbits(!estop_ && rt_active, !estop_ && lt_active);

    if (!estop_) {
      float rpm_mag  = (rt_active ? rt_n : (lt_active ? lt_n : 0.f)) * MAX_SPEED_RPM;
      float rpm_send = (rpm_mag > 0.f) ? std::max(50.f, rpm_mag) : 0.f; // min 50 rpm if moving
      drive_.set_speed_rpm(rpm_send);
      drive_.set_steering_deg(steer_cmd_deg);
    } else {
      drive_.set_speed_rpm(0.f);
    }

    // ----- Forks (RIGHT STICK Y): up->lift, down->lower -----
    const float ry_raw    = s.ry;           
    const float lift_cmd  = dz(-ry_raw);    // up to + (0..1)
    const float lower_cmd = dz( ry_raw);    // down to + (0..1)

    if (!estop_) {
      if (lift_cmd > FORK_DEADBAND && lift_cmd >= lower_cmd) {
        const int pwm = static_cast<int>(std::round(std::min(1.f, lift_cmd) * LIFT_PWM_MAX));
        fork_.lift_pwm(pwm);
        fork_status_ = "LIFT " + std::to_string(pwm) + "%";
      } else if (lower_cmd > FORK_DEADBAND && lower_cmd > lift_cmd) {
        const int valve = static_cast<int>(std::round(std::min(1.f, lower_cmd) * LOWER_VALVE_MAX));
        fork_.lower_valve(valve);
        fork_status_ = "LOWER " + std::to_string(valve);
      } else {
        fork_.stop_hydraulics();
        fork_status_ = "OFF";
      }
    } else {
      fork_.stop_hydraulics();
      fork_status_ = "OFF";
    }

    // Debug (0.5s throttle) showing raw trigger values too
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "ESTOP=%d  RT=%.2f LT=%.2f  rpm=%s  steer=%+.0f  RY=%+.2f  forks=%s",
      (int)estop_, rt_n, lt_n,
      (!estop_ ? "active" : "0"),
      steer_cmd_deg, ry_raw,
      fork_status_.c_str());
  }

  // Members
  forklift_control::JoyReader      joy_;
  forklift_control::SocketCanIface can_;
  forklift_control::DriveLogic     drive_;
  forklift_control::ForkLogic      fork_;
  forklift_control::SafetyMonitor  safety_;
  rclcpp::TimerBase::SharedPtr     ctrl_timer_;
  rclcpp::TimerBase::SharedPtr     ka_drive_timer_;
  rclcpp::TimerBase::SharedPtr     ka_fork_timer_;
  bool        estop_ = false;
  bool        first_joy_received_ = false;
  bool        startup_lockout_active_ = false;
  std::string fork_status_ = "OFF";

  bool inputs_neutral_(const forklift_control::JoyState& s) const {
    constexpr float STICK_THRESH = 0.15f;
    constexpr float TRIGGER_THRESH = 0.10f;

    if (std::fabs(s.lx) > STICK_THRESH) return false;
    if (std::fabs(s.ly) > STICK_THRESH) return false;
    if (std::fabs(s.rx) > STICK_THRESH) return false;
    if (std::fabs(s.ry) > STICK_THRESH) return false;
    if (s.lt > TRIGGER_THRESH) return false;
    if (s.rt > TRIGGER_THRESH) return false;

    if (s.A || s.B || s.X || s.Y ||
        s.LB || s.RB ||
        s.SELECT || s.START) {
      return false;
    }

    return true;
  }

  void apply_safe_outputs_() {
    drive_.set_direction_lowbits(false, false);
    drive_.set_speed_rpm(0.0f);
    drive_.set_steering_deg(0.0f);
    fork_.stop_hydraulics();
    fork_status_ = "OFF";
  }

  // zero-timer / one-shot behavior removed; tick_() handles recent-joy checks
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ForkliftControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
