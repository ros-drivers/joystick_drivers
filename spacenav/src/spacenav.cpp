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

/*
 * Author: Stuart Glaser
 */

#include "spacenav/spacenav.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "spnav.h" // NOLINT

#define SPACENAV_FULL_SCALE_PARAM_S "full_scale"

#define SPACENAV_ANGULAR_SCALE_PARAM_S "angular_scale"
#define SPACENAV_LINEAR_SCALE_PARAM_S "linear_scale"
#define SPACENAV_X_PARAM_S "/x"
#define SPACENAV_Y_PARAM_S "/y"
#define SPACENAV_Z_PARAM_S "/z"

#define SPACENAV_STATIC_COUNT_TRESHOLD_PARAM_S "static_count_threshold"
#define SPACENAV_ZERO_WHEN_STATIC_PARAM_S "zero_when_static"
#define SPACENAV_STATIC_TRANS_DEADBAND_PARAM_S "static_trans_deadband"
#define SPACENAV_STATIC_ROT_DEADBAND_PARAM_S "static_rot_deadband"
#define SPACENAV_USE_TWIST_STAMPED_PARAM_S "use_twist_stamped"

using namespace std::chrono_literals;

namespace spacenav
{

Spacenav::Spacenav(const rclcpp::NodeOptions & options)
: Node("spacenav", options)
{
  this->declare_parameter<double>(SPACENAV_FULL_SCALE_PARAM_S, 512.0);

  this->declare_parameter<double>(
    SPACENAV_ANGULAR_SCALE_PARAM_S SPACENAV_X_PARAM_S, 1);
  this->declare_parameter<double>(
    SPACENAV_ANGULAR_SCALE_PARAM_S SPACENAV_Y_PARAM_S, 1);
  this->declare_parameter<double>(
    SPACENAV_ANGULAR_SCALE_PARAM_S SPACENAV_Z_PARAM_S, 1);

  this->declare_parameter<double>(
    SPACENAV_LINEAR_SCALE_PARAM_S SPACENAV_X_PARAM_S, 1);
  this->declare_parameter<double>(
    SPACENAV_LINEAR_SCALE_PARAM_S SPACENAV_Y_PARAM_S, 1);
  this->declare_parameter<double>(
    SPACENAV_LINEAR_SCALE_PARAM_S SPACENAV_Z_PARAM_S, 1);

  // The number of polls needed to be done before the device is considered
  // "static"
  this->declare_parameter<int>(SPACENAV_STATIC_COUNT_TRESHOLD_PARAM_S, 30);
  // If true, the node will zero the output when the device is "static"
  this->declare_parameter<bool>(SPACENAV_ZERO_WHEN_STATIC_PARAM_S, true);
  // If the device is considered "static" and each trans, rot normed component
  // is below the deadband, it will output zeros in either rotation,
  // translation, or both.
  this->declare_parameter<double>(SPACENAV_STATIC_TRANS_DEADBAND_PARAM_S, 0.1);
  this->declare_parameter<double>(SPACENAV_STATIC_ROT_DEADBAND_PARAM_S, 0.1);
  use_twist_stamped = this->declare_parameter<bool>(SPACENAV_USE_TWIST_STAMPED_PARAM_S, false);

  auto param_change_callback = [this](
    std::vector<rclcpp::Parameter> parameters) {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      // Parameters
      for (const auto & parameter : parameters) {
        if (parameter.get_name() == SPACENAV_FULL_SCALE_PARAM_S) {
          if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
            parameter.as_double() < 1e-10)
          {
            result.successful = false;
            result.reason = "full_scale < 1e-10";
          }
        }
      }
      return result;
    };
  callback_handler =
    this->add_on_set_parameters_callback(param_change_callback);

  // Setup publishers and Timer
  publisher_offset = this->create_publisher<geometry_msgs::msg::Vector3>(
    "spacenav/offset", 10);
  publisher_rot_offset = this->create_publisher<geometry_msgs::msg::Vector3>(
    "spacenav/rot_offset", 10);
  if (use_twist_stamped) {
    publisher_twist_stamped =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("spacenav/twist_stamped", 10);
  } else {
    publisher_twist =
      this->create_publisher<geometry_msgs::msg::Twist>("spacenav/twist", 10);
  }
  publisher_joy =
    this->create_publisher<sensor_msgs::msg::Joy>("spacenav/joy", 10);

  timer_ =
    this->create_wall_timer(1ms, std::bind(&Spacenav::poll_spacenav, this));

  RCLCPP_DEBUG(
    get_logger(), "full scale: %.1f", full_scale);
  RCLCPP_DEBUG(
    get_logger(), "linear_scale: %.3f %.3f %.3f", linear_scale[0],
    linear_scale[1], linear_scale[2]);
  RCLCPP_DEBUG(
    get_logger(), "angular_scale: %.3f %.3f %.3f", angular_scale[0],
    angular_scale[1], angular_scale[2]);
  RCLCPP_DEBUG(
    get_logger(), "static_count_threshold: %df",
    static_count_threshold);
  RCLCPP_DEBUG(
    get_logger(), "zero_when_static : %d", zero_when_static);
  RCLCPP_DEBUG(
    get_logger(), "static_trans_deadband: %.3f",
    static_trans_deadband);
  RCLCPP_DEBUG(
    get_logger(), "static_rot_deadband: %.3f", static_rot_deadband);
}

Spacenav::~Spacenav()
{
  if (spacenav_is_open) {
    spnav_close();
  }
}

void Spacenav::poll_spacenav()
{
  if (!spacenav_is_open) {
    if (spnav_open() == -1) {
      RCLCPP_ERROR(
        get_logger(),
        "Could not open the space navigator device. "
        "Did you remember to run spacenavd (as root)?");
      return;
    } else {
      spacenav_is_open = true;
    }
  }

  this->get_parameter<double>(
    SPACENAV_FULL_SCALE_PARAM_S, full_scale);
  this->get_parameter<double>(
    SPACENAV_LINEAR_SCALE_PARAM_S SPACENAV_X_PARAM_S,
    linear_scale[0]);
  this->get_parameter<double>(
    SPACENAV_LINEAR_SCALE_PARAM_S SPACENAV_Y_PARAM_S,
    linear_scale[1]);
  this->get_parameter<double>(
    SPACENAV_LINEAR_SCALE_PARAM_S SPACENAV_Z_PARAM_S,
    linear_scale[2]);

  this->get_parameter<double>(
    SPACENAV_ANGULAR_SCALE_PARAM_S SPACENAV_X_PARAM_S,
    angular_scale[0]);
  this->get_parameter<double>(
    SPACENAV_ANGULAR_SCALE_PARAM_S SPACENAV_Y_PARAM_S,
    angular_scale[1]);
  this->get_parameter<double>(
    SPACENAV_ANGULAR_SCALE_PARAM_S SPACENAV_Z_PARAM_S,
    angular_scale[2]);

  this->get_parameter<int>(
    SPACENAV_STATIC_COUNT_TRESHOLD_PARAM_S,
    static_count_threshold);
  this->get_parameter<bool>(
    SPACENAV_ZERO_WHEN_STATIC_PARAM_S,
    zero_when_static);
  this->get_parameter<double>(
    SPACENAV_STATIC_TRANS_DEADBAND_PARAM_S,
    static_trans_deadband);
  this->get_parameter<double>(
    SPACENAV_STATIC_ROT_DEADBAND_PARAM_S,
    static_rot_deadband);

  bool queue_empty = false;
  while (!queue_empty) {
    auto msg_joystick = std::make_unique<sensor_msgs::msg::Joy>();
    msg_joystick->header.stamp = get_clock()->now();
    // Output changes each time a button event happens, or when a motion
    // event happens and the queue is empty.

    switch (spnav_poll_event(&sev)) {
      case 0:
        queue_empty = true;

        if (++no_motion_count > static_count_threshold) {
          if (zero_when_static ||
            (fabs(normed_vx) < static_trans_deadband &&
            fabs(normed_vy) < static_trans_deadband &&
            fabs(normed_vz) < static_trans_deadband))
          {
            normed_vx = normed_vy = normed_vz = 0;
          }

          if (zero_when_static ||
            (fabs(normed_wx) < static_rot_deadband &&
            fabs(normed_wy) < static_rot_deadband &&
            fabs(normed_wz) < static_rot_deadband))
          {
            normed_wx = normed_wy = normed_wz = 0;
          }
          no_motion_count = 0;
          motion_stale = true;
        }
        break;

      case SPNAV_EVENT_MOTION:
        normed_vx = sev.motion.z / full_scale;
        normed_vy = -sev.motion.x / full_scale;
        normed_vz = sev.motion.y / full_scale;

        normed_wx = sev.motion.rz / full_scale;
        normed_wy = -sev.motion.rx / full_scale;
        normed_wz = sev.motion.ry / full_scale;

        motion_stale = true;
        break;

      case SPNAV_EVENT_BUTTON:

        if (sev.button.bnum < 0) {
          RCLCPP_WARN(
            get_logger(), "Negative spacenav buttons not supported. Got %i", sev.button.bnum);
          break;
        }
        if (sev.button.bnum < static_cast<int>(joystick_buttons.size())) {
          // Update known buttons
          joystick_buttons[sev.button.bnum] = sev.button.press;
        } else {
          // Enlarge, fill up with zeros, and support the new button
          joystick_buttons.resize(sev.button.bnum + 1, 0);
          joystick_buttons[sev.button.bnum] = sev.button.press;
        }
        joy_stale = true;
        break;

      default:
        RCLCPP_WARN(
          get_logger(),
          "Unknown message type in spacenav. This should never happen.");
        break;
    }

    if (motion_stale && (queue_empty || joy_stale)) {
      // The offset and rot_offset are scaled.
      auto msg_offset = std::make_unique<geometry_msgs::msg::Vector3>();
      msg_offset->x = normed_vx * linear_scale[0];
      msg_offset->y = normed_vy * linear_scale[1];
      msg_offset->z = normed_vz * linear_scale[2];

      auto msg_rot_offset = std::make_unique<geometry_msgs::msg::Vector3>();
      msg_rot_offset->x = normed_wx * angular_scale[0];
      msg_rot_offset->y = normed_wy * angular_scale[1];
      msg_rot_offset->z = normed_wz * angular_scale[2];

      auto msg_twist = std::make_unique<geometry_msgs::msg::Twist>();
      msg_twist->linear = *msg_offset;
      msg_twist->angular = *msg_rot_offset;

      publisher_offset->publish(std::move(msg_offset));
      publisher_rot_offset->publish(std::move(msg_rot_offset));
      if (use_twist_stamped) {
        auto msg_twist_stamped = std::make_unique<geometry_msgs::msg::TwistStamped>();
        msg_twist_stamped->header.stamp = msg_joystick->header.stamp;
        msg_twist_stamped->twist = *msg_twist;
        publisher_twist_stamped->publish(std::move(msg_twist_stamped));
      } else {
        publisher_twist->publish(std::move(msg_twist));
      }

      no_motion_count = 0;
      motion_stale = false;
      joy_stale = true;
    }
    if (joy_stale) {
      msg_joystick->axes.resize(6);
      // The joystick.axes are normalized within [-1, 1].
      msg_joystick->axes[0] = normed_vx;
      msg_joystick->axes[1] = normed_vy;
      msg_joystick->axes[2] = normed_vz;
      msg_joystick->axes[3] = normed_wx;
      msg_joystick->axes[4] = normed_wy;
      msg_joystick->axes[5] = normed_wz;
      msg_joystick->buttons = joystick_buttons;
      publisher_joy->publish(std::move(msg_joystick));
    }
  }
}

}  // namespace spacenav

RCLCPP_COMPONENTS_REGISTER_NODE(spacenav::Spacenav)
