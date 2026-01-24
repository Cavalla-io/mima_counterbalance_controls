#pragma once
#include <atomic>
#include <thread>
#include <functional>
#include <string>
#include <array>
#include <stdexcept>
#include <cstring>
#include <cerrno>

// Linux headers
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "forklift_control/DriveLogic.hpp"  // for ICanIface, CanFrame

namespace forklift_control {

class SocketCanIface : public ICanIface {
public:
  explicit SocketCanIface(const std::string& ifname)
  : ifname_(ifname)
  {
    try_open_socket_();
  }

  ~SocketCanIface() override {
    running_.store(false);
    if (sock_ >= 0) ::shutdown(sock_, SHUT_RD);
    if (rx_thread_.joinable()) rx_thread_.join();
    if (sock_ >= 0) ::close(sock_);
  }

  // Returns true if CAN bus is currently operational
  bool is_ok() const { return can_ok_.load(); }

  // Attempt to reconnect to the CAN bus (call periodically when is_ok() is false)
  bool try_reconnect() {
    if (can_ok_.load()) return true;

    // Close existing socket if any
    if (sock_ >= 0) {
      running_.store(false);
      ::shutdown(sock_, SHUT_RD);
      if (rx_thread_.joinable()) rx_thread_.join();
      ::close(sock_);
      sock_ = -1;
    }

    return try_open_socket_();
  }

  // Send one classic CAN frame (8 bytes)
  void send(uint32_t arb_id, const std::array<uint8_t,8>& data, uint8_t dlc = 8) override {
    if (!can_ok_.load() || sock_ < 0) return;  // Silently skip if CAN is down

    struct can_frame cf {};
    cf.can_id  = arb_id;
    cf.can_dlc = (dlc <= 8) ? dlc : 8;
    std::memcpy(cf.data, data.data(), cf.can_dlc);
    ssize_t n = ::write(sock_, &cf, sizeof(cf));
    if (n != sizeof(cf)) {
      can_ok_.store(false);  // Mark CAN as down instead of throwing
    }
  }

  void register_rx(const std::function<void(const CanFrame&)>& cb) override {
    rx_cb_ = cb;
  }

private:
  bool try_open_socket_() {
    // Open RAW CAN socket
    sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
      can_ok_.store(false);
      return false;
    }

    // Resolve interface index
    struct ifreq ifr {};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname_.c_str());
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
      ::close(sock_);
      sock_ = -1;
      can_ok_.store(false);
      return false;
    }

    // Bind socket to interface
    sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      ::close(sock_);
      sock_ = -1;
      can_ok_.store(false);
      return false;
    }

    can_ok_.store(true);
    running_.store(true);
    rx_thread_ = std::thread([this]{ this->rx_loop_(); });
    return true;
  }

  void rx_loop_() {
    while (running_.load()) {
      struct can_frame cf {};
      ssize_t n = ::read(sock_, &cf, sizeof(cf));
      if (n < 0) {
        if (errno == EINTR) continue;
        // If socket is shutting down, exit quietly
        if (!running_.load()) break;
        // Mark CAN as down on read errors (except EINTR)
        if (errno == ENETDOWN || errno == ENODEV) {
          can_ok_.store(false);
        }
        continue;
      }
      if (static_cast<size_t>(n) < sizeof(cf)) continue;

      // Convert to our generic frame and emit
      if (rx_cb_) {
        CanFrame f{};
        f.id  = cf.can_id & CAN_EFF_MASK;   // works for std IDs; extend if you need EFF flags
        f.dlc = cf.can_dlc;
        std::memcpy(f.data.data(), cf.data, f.dlc);
        rx_cb_(f);
      }
    }
  }

private:
  std::string ifname_;
  int sock_{-1};
  std::atomic<bool> running_{false};
  std::atomic<bool> can_ok_{false};
  std::thread rx_thread_;
  std::function<void(const CanFrame&)> rx_cb_;
};

} // namespace forklift_control
