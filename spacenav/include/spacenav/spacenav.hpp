#ifndef SPACENAV__SPACENAV_HPP_
#define SPACENAV__SPACENAV_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "spnav.h" // NOLINT

namespace spacenav
{

class Spacenav final : public rclcpp::Node
{
public:
  explicit Spacenav(const rclcpp::NodeOptions & options);

  ~Spacenav();

private:
  void poll_spacenav();

  OnSetParametersCallbackHandle::SharedPtr callback_handler;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_offset;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_rot_offset;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_twist;

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_joy;

  bool spacenav_is_open = false;

  double full_scale;
  double linear_scale[3];
  double angular_scale[3];

  int static_count_threshold;
  bool zero_when_static;
  double static_trans_deadband;
  double static_rot_deadband;

  spnav_event sev;
  bool joy_stale = false;
  int no_motion_count = 0;
  bool motion_stale = false;
  double normed_vx = 0;
  double normed_vy = 0;
  double normed_vz = 0;
  double normed_wx = 0;
  double normed_wy = 0;
  double normed_wz = 0;
};

}  // namespace spacenav

#endif  // SPACENAV__SPACENAV_HPP_
