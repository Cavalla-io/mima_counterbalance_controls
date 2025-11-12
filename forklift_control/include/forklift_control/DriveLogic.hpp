#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <functional>

namespace forklift_control {

// Minimal generic CAN types
struct CanFrame {
  uint32_t id = 0;
  std::array<uint8_t,8> data{};
  uint8_t dlc = 8;
};

class ICanIface {
public:
  virtual ~ICanIface() = default;
  virtual void send(uint32_t arb_id, const std::array<uint8_t,8>& data, uint8_t dlc = 8) = 0;
  virtual void register_rx(const std::function<void(const CanFrame&)>& cb) = 0; // optional for feedback
};

class DriveLogic {
public:
  // node_id: COB-ID will be 0x200 + node_id. If not given, defaults to 3.
  explicit DriveLogic(ICanIface& can_iface,
                      std::optional<int> node_id = std::nullopt,
                      std::optional<float> wheel_circ_m = std::nullopt,
                      std::optional<float> gear_ratio  = std::nullopt,
                      uint8_t byte0_base = 0x01);

  // ---- Driving fields (write -> immediate send) ----
  void set_speed_rpm(float rpm_nonneg);      // magnitude only; direction via set_direction_lowbits()
  void set_steering_deg(float deg);          // -120.00..+120.00 deg (scaled 0.01/LSB -> -12000..12000)
  void set_ramps(std::optional<float> accel_time_s,
                 std::optional<float> decel_time_s); // each 0.1..25.5s

  void set_direction_lowbits(bool rt_active, bool lt_active); // RT:0x02, LT:0x04 (RT wins if both)
  void set_lift_solenoid(bool on);   // bit5 (0x20)
  void set_lower_solenoid(bool on);  // bit4 (0x10)
  void set_descent_valve(int value_0_200);   // Byte7

  // Convenience
  void send_now();                    // pack current fields and send
  int  node_id() const { return node_id_; }
  uint32_t cob_id() const { return cob_cmd_; }

private:
  // helpers
  void pack_and_send_0x200_();       // builds 8 bytes (per your Python) and sends
  static int clampi(int v, int lo, int hi);
  static float clampf(float v, float lo, float hi);
  static void put_i16_le(std::array<uint8_t,8>& b, int off, int16_t v);

private:
  ICanIface& can_;

  // config
  int node_id_ = 3;
  uint32_t cob_cmd_ = 0x203;           // 0x200 + node_id
  std::optional<float> wheel_circ_m_;
  std::optional<float> gear_ratio_;
  uint8_t b0_base_ = 0x01;             // interlock

  // byte0 submasks
  uint8_t dir_mask_ = 0x00;            // 0x02 RT, 0x04 LT
  uint8_t hyd_mask_ = 0x00;            // 0x10 lower, 0x20 lift

  // other payload fields
  int16_t speed_rpm_ = 0;              // magnitude only (>=0)
  float   accel_time_s_ = 1.0f;        // 0.1..25.5
  float   decel_time_s_ = 1.0f;        // 0.1..25.5
  int16_t steer_counts_ = 0;           // -12000..+12000 (0.01 deg/LSB)
  uint8_t descent_ = 0;                // 0..200
};

} // namespace forklift_control
