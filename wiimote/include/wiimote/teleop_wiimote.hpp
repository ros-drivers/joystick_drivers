// Copyright 2020 Intel Corporation
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

/*
 * ROS Node for using a wiimote control unit to direct a robot.
 */

/*
 * Initial C++ implementation by
 *   Mark Horn <mark.d.horn@intel.com>
 *
 * Revisions:
 *
 */

#pragma once
#ifndef WIIMOTE__TELEOP_WIIMOTE_HPP_
#define WIIMOTE__TELEOP_WIIMOTE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <wiimote_msgs/msg/state.hpp>

class TeleopWiimote : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * \brief rclcpp component-compatible constructor
   * \param options
   */
  explicit TeleopWiimote(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

private:
  void rumble_feedback(std::chrono::milliseconds duration);
  void set_led_feedback(double value);
  void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr & joy);
  void wiimote_state_callback(const wiimote_msgs::msg::State::ConstSharedPtr & wiistate);

  double linear_x_max_velocity_;   // m/s
  double linear_x_min_velocity_;   // m/s
  double angular_z_max_velocity_;  // rad/s
  double angular_z_min_velocity_;  // rad/s

  double percent_linear_throttle_;   // 0.0 - 1.0 (1.0 = 100%)
  double percent_angular_throttle_;  // 0.0 - 1.0 (1.0 = 100%)

  rclcpp::Logger logger_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JoyFeedbackArray>::SharedPtr joy_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<wiimote_msgs::msg::State>::SharedPtr wiimote_sub_;

  bool dpad_in_use_;
  bool njoy_in_use_;
};

#endif  // WIIMOTE__TELEOP_WIIMOTE_HPP_
