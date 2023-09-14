/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SPACENAV__SPACENAV_HPP_
#define SPACENAV__SPACENAV_HPP_

#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
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

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_twist_stamped;

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
  bool use_twist_stamped;

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

  // We'll resize dynamically to support spacenav devices with more buttons.
  std::vector<int> joystick_buttons = {0, 0};
};

}  // namespace spacenav

#endif  // SPACENAV__SPACENAV_HPP_
