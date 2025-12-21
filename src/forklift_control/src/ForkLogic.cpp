#include "forklift_control/ForkLogic.hpp"
#include <cmath>

namespace forklift_control {

ForkLogic::ForkLogic(ICanIface& can_iface, DriveLogic& drive, std::optional<int> node_id)
: can_(can_iface), drive_(drive)
{
  node_id_  = node_id.has_value() ? clampi(*node_id, 0, 0x7F) : drive.node_id();
  cob_pump_ = 0x300u + static_cast<uint32_t>(node_id_);
  send_0x300_(); // neutral
}

// ---- High-level API ----
void ForkLogic::lift_pwm(int pwm_0_100)
{
  int pwm = clampi(pwm_0_100, 0, 100);
  drive_.set_lower_solenoid(false);
  drive_.set_descent_valve(0);
  drive_.set_lift_solenoid(true);
  set_pump_pwm(pwm);  // sends 0x300
}

void ForkLogic::lower_valve(int valve_0_200)
{
  int valve = clampi(valve_0_200, 0, 200);
  set_pump_pwm(0);                 // sends 0x300
  drive_.set_lift_solenoid(false);
  drive_.set_lower_solenoid(true);
  drive_.set_descent_valve(valve); // DriveLogic sends 0x200
}

void ForkLogic::stop_hydraulics()
{
  set_pump_pwm(0);       // sends 0x300
  drive_.set_descent_valve(0);
  drive_.set_lift_solenoid(false);
  drive_.set_lower_solenoid(false);
  
  // also clear aux bits? Standard practice usually yes for safety
  attach2l_ = false; attach2r_ = false;
  attach3l_ = false; attach3r_ = false;
  // trigger update if needed, but set_pump_pwm(0) just sent one.
  // we should resend if we changed bits.
  send_0x300_();
}

void ForkLogic::set_tilt(int pwm_speed)
{
  int pwm = clampi(pwm_speed, -100, 100);
  attach2r_ = (pwm > 0);
  attach2l_ = (pwm < 0);
  set_pump_pwm(std::abs(pwm)); // sends 0x300
}

void ForkLogic::set_sideshift(int pwm_speed)
{
  int pwm = clampi(pwm_speed, -100, 100);
  attach3r_ = (pwm > 0);
  attach3l_ = (pwm < 0);
  set_pump_pwm(std::abs(pwm)); // sends 0x300
}

// ---- Low-level setters ----
void ForkLogic::set_pump_pwm(int pwm_0_100)
{
  pump_pwm_0_100_ = clampi(pwm_0_100, 0, 100);
  send_0x300_();
}

void ForkLogic::set_pump_ramps(std::optional<float> accel_time_s,
                               std::optional<float> decel_time_s)
{
  if (accel_time_s.has_value())
    pump_accel_s_ = std::min(25.5f, std::max(0.1f, *accel_time_s));
  if (decel_time_s.has_value())
    pump_decel_s_ = std::min(25.5f, std::max(0.1f, *decel_time_s));
  send_0x300_();
}

void ForkLogic::keepalive()
{
  send_0x300_();
}

// ---- pack & send ----
void ForkLogic::send_0x300_()
{
  if (!safe_state_) {
    return;  // suppress CAN traffic while unsafe
  }

  std::array<uint8_t,8> d{};
  // Byte0: attachments
  uint8_t b0 = 0;
  b0 |= static_cast<uint8_t>(attach2l_ ? 1u : 0u) << 0;
  b0 |= static_cast<uint8_t>(attach2r_ ? 1u : 0u) << 1;
  b0 |= static_cast<uint8_t>(attach3l_ ? 1u : 0u) << 2;
  b0 |= static_cast<uint8_t>(attach3r_ ? 1u : 0u) << 3;
  d[0] = b0;

  d[1] = 0;                                        // reserved (Battery Level)
  d[2] = 0;                                        // reserved (Was PWM)

  // Byte3-4: Pump Speed RPM (0-5000) mapped from pump_pwm_0_100_
  // 100% -> 5000 RPM => multiplier 50
  uint16_t rpm = static_cast<uint16_t>(pump_pwm_0_100_ * 50);
  d[3] = static_cast<uint8_t>(rpm & 0xFF);
  d[4] = static_cast<uint8_t>((rpm >> 8) & 0xFF);

  // Byte5: pump accel (0.1..25.5 -> 1..255)
  int accel_u8 = clampi(static_cast<int>(std::lround(pump_accel_s_ * 10.f)), 1, 255);
  d[5] = static_cast<uint8_t>(accel_u8);

  // Byte6: pump decel
  int decel_u8 = clampi(static_cast<int>(std::lround(pump_decel_s_ * 10.f)), 1, 255);
  d[6] = static_cast<uint8_t>(decel_u8);
  
  d[7] = 0;                                        // reserved

  can_.send(cob_pump_, d, 8);
}

void ForkLogic::set_safe_state(bool safe)
{
  if (safe == safe_state_) return;

  if (!safe && safe_state_) {
    // send a neutral frame before disabling future transmissions
    stop_hydraulics();
  }

  safe_state_ = safe;

  if (safe_state_) {
    send_0x300_();
  }
}

} // namespace forklift_control
