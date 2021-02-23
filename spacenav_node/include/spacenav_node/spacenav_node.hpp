#ifndef SPACENAV_NODE__SPACENAV_NODE_HPP_
#define SPACENAV_NODE__SPACENAV_NODE_HPP_
// Author: Hye-jong KIM

#include <cstdio>
#include <vector>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp/publisher.hpp"

#include "spnav.h"  // NOLINT

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace spacenav_node
{

using namespace std::chrono_literals;

using Vector3 = geometry_msgs::msg::Vector3;
using Twist = geometry_msgs::msg::Twist;
using Joy = sensor_msgs::msg::Joy;

class SpacenavNode : public rclcpp::Node
{
public:
  explicit SpacenavNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~SpacenavNode();

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<Vector3>::SharedPtr offset_pub_;
  rclcpp::Publisher<Vector3>::SharedPtr rot_offset_pub_;
  rclcpp::Publisher<Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<Joy>::SharedPtr joy_pub_;

  double full_scale_;

  std::vector<double> linear_scale_;
  std::vector<double> angular_scale_;

  int static_count_threshold_;
  bool zero_when_static_;
  double static_trans_deadband_;
  double static_rot_deadband_;

  Joy joystick_msg_;

  spnav_event sev_;
  int no_motion_count_ = 0;
  bool motion_stale_ = false;
  bool joy_stale_ = false;
  bool queue_empty_ = false;
  double normed_vx_ = 0;
  double normed_vy_ = 0;
  double normed_vz_ = 0;
  double normed_wx_ = 0;
  double normed_wy_ = 0;
  double normed_wz_ = 0;

  bool ensureThreeComponents(std::vector<double> & param);
};

}  // namespace spacenav_node

#endif  // SPACENAV_NODE__SPACENAV_NODE_HPP_
