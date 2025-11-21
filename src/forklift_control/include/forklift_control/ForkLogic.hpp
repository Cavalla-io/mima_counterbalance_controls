#pragma once
#include <array>
#include <cstdint>
#include <optional>

#include "forklift_control/DriveLogic.hpp"  // for ICanIface, DriveLogic

namespace forklift_control {

class ForkLogic {
public:
  // If node_id is not provided, we use drive.node_id()
  ForkLogic(ICanIface& can_iface, DriveLogic& drive, std::optional<int> node_id = std::nullopt);

  // High-level API (matches Python)
  void lift_pwm(int pwm_0_100);         // LIFT: lift solenoid ON, descent=0, pump=PWM
  void lower_valve(int valve_0_200);    // LOWER: pump=0, lower solenoid ON, descent=valve
  void stop_hydraulics();               // STOP: pump=0, both solenoids OFF, descent=0
  
  // Low-level setters
  void set_pump_pwm(int pwm_0_100);     // 0..100
  void set_pump_ramps(std::optional<float> accel_time_s,
                      std::optional<float> decel_time_s); // each 0.1..25.5 s
  void keepalive();                     // resend last 0x300 frame
  void set_safe_state(bool safe);       // mask outputs when unsafe

  // Optional: attach bits if you need them later
  void set_attach2l(bool on) { attach2l_ = on; }
  void set_attach2r(bool on) { attach2r_ = on; }
  void set_attach3l(bool on) { attach3l_ = on; }
  void set_attach3r(bool on) { attach3r_ = on; }

  int  node_id() const { return node_id_; }
  uint32_t cob_id() const { return cob_pump_; }

private:
  static inline int clampi(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
  }

  void send_0x300_(); // packs current state and sends

private:
  ICanIface& can_;
  DriveLogic& drive_;

  int node_id_ = 3;
  uint32_t cob_pump_ = 0x303;

  // 0x300 payload fields + flags
  bool attach2l_ = false;
  bool attach2r_ = false;
  bool attach3l_ = false;
  bool attach3r_ = false;

  int   pump_pwm_0_100_   = 0;    // Byte2
  int   ac_pump_rpm_      = 0;    // Byte3 (kept for parity; unused -> 0)
  float pump_accel_s_     = 1.0f; // Byte4 (0.1..25.5 -> 1..255)
  float pump_decel_s_     = 1.0f; // Byte6 (0.1..25.5 -> 1..255)

  bool  safe_state_       = true;
};

} // namespace forklift_control
