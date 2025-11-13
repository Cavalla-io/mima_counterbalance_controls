#include "forklift_control/DriveLogic.hpp"
#include <cmath>
#include <iostream>

namespace forklift_control {

int DriveLogic::clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
float DriveLogic::clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
void DriveLogic::put_i16_le(std::array<uint8_t,8>& b, int off, int16_t v) {
  b[off+0] = static_cast<uint8_t>(v & 0xFF);
  b[off+1] = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFF);
}

DriveLogic::DriveLogic(ICanIface& can_iface,
                       std::optional<int> node_id,
                       std::optional<float> wheel_circ_m,
                       std::optional<float> gear_ratio,
                       uint8_t byte0_base)
: can_(can_iface),
  wheel_circ_m_(wheel_circ_m),
  gear_ratio_(gear_ratio),
  b0_base_(byte0_base)
{
  if (node_id.has_value()) node_id_ = clampi(*node_id, 0, 0x7F);  // conservative
  cob_cmd_ = 0x200u + static_cast<uint32_t>(node_id_);
  // optional: can_.register_rx(...) if you want to parse feedback frames
  pack_and_send_0x200_(); // neutral on startup
}

void DriveLogic::set_speed_rpm(float rpm_nonneg)
{
  // magnitude only (direction is in low bits of Byte0)
  int r = static_cast<int>(std::lround(std::fabs(rpm_nonneg)));
  r = clampi(r, 0, 4000);
  speed_rpm_ = static_cast<int16_t>(r);
  pack_and_send_0x200_();
}

void DriveLogic::set_steering_deg(float deg)
{
  // scale 0.01 deg/LSB
  int raw = static_cast<int>(std::lround(deg * 100.0f));
  raw = clampi(raw, -12000, 12000);
  steer_counts_ = static_cast<int16_t>(raw);
  pack_and_send_0x200_();
}

void DriveLogic::set_ramps(std::optional<float> accel_time_s, std::optional<float> decel_time_s)
{
  if (accel_time_s.has_value()) accel_time_s_ = clampf(*accel_time_s, 0.1f, 25.5f);
  if (decel_time_s.has_value()) decel_time_s_ = clampf(*decel_time_s, 0.1f, 25.5f);
  pack_and_send_0x200_();
}

void DriveLogic::set_direction_lowbits(bool rt_active, bool lt_active)
{
  // RT wins if both
  dir_mask_ = rt_active ? 0x02 : (lt_active ? 0x04 : 0x00);
  pack_and_send_0x200_();
}

void DriveLogic::set_lift_solenoid(bool on)
{
  if (on) hyd_mask_ |= 0x20; else hyd_mask_ &= static_cast<uint8_t>(~0x20);
  pack_and_send_0x200_();
}

void DriveLogic::set_lower_solenoid(bool on)
{
  if (on) hyd_mask_ |= 0x10; else hyd_mask_ &= static_cast<uint8_t>(~0x10);
  pack_and_send_0x200_();
}

void DriveLogic::set_descent_valve(int value_0_200)
{
  descent_ = static_cast<uint8_t>(clampi(value_0_200, 0, 200));
  pack_and_send_0x200_();
}

void DriveLogic::send_now()
{
  pack_and_send_0x200_();
}

void DriveLogic::pack_and_send_0x200_()
{
  std::array<uint8_t,8> d{};
  // Byte0
  uint8_t byte0 = static_cast<uint8_t>(b0_base_ & static_cast<uint8_t>(~0x01u));
  byte0 = static_cast<uint8_t>((byte0 | dir_mask_ | hyd_mask_) & 0xFF);
  if (safe_state_) {
    byte0 |= 0x01u;
  } else {
    byte0 &= static_cast<uint8_t>(~0x01u);
  }
  d[0] = byte0;
  // Bytes1-2: speed int16 LE (non-negative)
  put_i16_le(d, 1, speed_rpm_);
  // Byte3 accel (0.1..25.5 s → 1..255)
  int accel_u8 = clampi(static_cast<int>(std::lround(accel_time_s_ * 10.0f)), 1, 255);
  d[3] = static_cast<uint8_t>(accel_u8);
  // Byte4 decel
  int decel_u8 = clampi(static_cast<int>(std::lround(decel_time_s_ * 10.0f)), 1, 255);
  d[4] = static_cast<uint8_t>(decel_u8);
  // Bytes5-6: steer int16 LE
  put_i16_le(d, 5, steer_counts_);
  // Byte7: descent 0..200
  d[7] = descent_;

  // Debug (optional)
  // std::fprintf(stderr, "[drive] TX %03X  b0=%02X spd=%d acc=%u dec=%u steer=%d desc=%u\n",
  //              cob_cmd_, d[0], speed_rpm_, d[3], d[4], steer_counts_, d[7]);

  can_.send(cob_cmd_, d, 8);
}

void DriveLogic::set_safe_state(bool safe)
{
  if (safe == safe_state_) return;
  safe_state_ = safe;
  pack_and_send_0x200_();
}

} // namespace forklift_control
