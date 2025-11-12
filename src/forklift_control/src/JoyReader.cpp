/*
 This file literally just reads the joystick and publishes 
 the state to a topic. that's it. real af
*/

#include "forklift_control/JoyReader.hpp"

using sensor_msgs::msg::Joy;

namespace forklift_control {

JoyReader::JoyReader(rclcpp::Node& node,
                     const std::string& topic,
                     const rclcpp::QoS& qos)
{
  sub_ = node.create_subscription<Joy>(
    topic, qos,
    std::bind(&JoyReader::cb_, this, std::placeholders::_1));
}

void JoyReader::cb_(const Joy::SharedPtr msg)
{
  auto axis = [&](size_t i){ return (i < msg->axes.size()) ? msg->axes[i] : 0.0f; };
  auto btn  = [&](size_t i){ return (i < msg->buttons.size()) ? msg->buttons[i] : 0; };

  JoyState s;
  s.lx = axis(0);
  s.ly = axis(1);
  s.rx = axis(2);
  s.ry = axis(3);
  s.lt = axis(4);
  s.rt = axis(5);
  s.A  = btn(0);
  s.B  = btn(1);
  s.X  = btn(2);
  s.Y  = btn(3);
  s.START  = btn(6);
  s.SELECT = btn(8);
  s.LB = btn(9);
  s.RB = btn(10);

  std::lock_guard<std::mutex> lk(mtx_);
  state_ = s;
  have_data_ = true;
}

JoyState JoyReader::latest() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return state_;
}

bool JoyReader::has_message() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return have_data_;
}

} // namespace forklift_control
