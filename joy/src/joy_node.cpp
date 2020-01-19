// Copyright 2009, 2020, Willow Garage, Inc., Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <dirent.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>

#include <cstring>
#include <string>

#include "joy/joy_node.hpp"

namespace joy
{

JoyNode::JoyNode(rclcpp::NodeOptions options)
: Node("joy node", options),
  joy_pub(this->create_publisher<sensor_msgs::msg::Joy>("joy", 1)),
  feedback_sub(this->create_subscription<sensor_msgs::msg::JoyFeedbackArray>(
      "joy/set_feedback", 10, std::bind(&JoyNode::feedback_callback, this, std::placeholders::_1)))
{
  dev_path = this->declare_parameter("dev", "/dev/input/js0");
  dev_ff_path = this->declare_parameter(
    "ff_dev",
    "/dev/input/by-id/usb-Sony_PLAYSTATION_R_3_Controller-event-joystick");
  dev_name = this->declare_parameter("dev_name", "");
  deadzone = this->declare_parameter("deadzone", 0.05);
  autorepeat_rate = this->declare_parameter("autorepeat_rate", 0);
  coalesce_interval = this->declare_parameter("coalesce_interval", 0.001);
  // TODO(jwhitleyastuff): Make "default_trig_val actually do what we expect.
  sticky_buttons = this->declare_parameter("sticky_buttons", false);

  if (!dev_name.empty()) {
    std::string path = get_dev_path_by_name(dev_name);

    if (path.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Couldn't find a joystick with name %s. Falling back to device path.",
        dev_name.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Using %s as joystick device.", path.c_str());
      dev_path = path;
    }
  }

  if (autorepeat_rate > (1.0 / coalesce_interval)) {
    RCLCPP_WARN(
      this->get_logger(),
      "autorepeat_rate (%f Hz) > 1 / coalesce_interval (%f Hz) does not make sense. "
      "Timing behavior is not well defined.",
      autorepeat_rate,
      1.0 / coalesce_interval);
  }

  if (deadzone >= 1) {
    RCLCPP_WARN(
      this->get_logger(),
      "deadzone (%f) greater than 1.0, setting to 0.9. "
      "If this was not intended, see the docs.",
      deadzone);
    deadzone = 0.9;
  } else if (deadzone > 0.9) {
    RCLCPP_WARN(
      this->get_logger(),
      "deadzone (%f) greater than 0.9, setting to 0.9.",
      deadzone);
    deadzone = 0.9;
  } else if (deadzone < 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "deadzone (%f) less than 0, setting to 0.",
      deadzone);
    deadzone = 0;
  }

  if (autorepeat_rate < 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "autorepeat_rate (%f) less than 0, setting to 0.",
      autorepeat_rate);
    autorepeat_rate = 0;
  }

  if (coalesce_interval < 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "coalesce_interval (%f) less than 0, setting to 0.",
      coalesce_interval);
    coalesce_interval = 0;
  }
}

void JoyNode::feedback_callback(const sensor_msgs::msg::JoyFeedbackArray::SharedPtr msg)
{
  (void)msg;
}

std::string JoyNode::get_dev_path_by_name(const std::string name)
{
  const char path[] = "/dev/input";  // no trailing / here
  struct dirent * entry;
  struct stat stat_buf;

  DIR * dev_dir = opendir(path);

  if (dev_dir == NULL) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Couldn't open %s. Error %i: %s.",
      path, errno, strerror(errno));
    return "";
  }

  while ((entry = readdir(dev_dir)) != NULL) {
    // filter entries
    if (std::strncmp(entry->d_name, "js", 2) != 0) {  // skip device if it's not a joystick
      continue;
    }

    std::string current_path = std::string(path) + "/" + entry->d_name;

    if (stat(current_path.c_str(), &stat_buf) == -1) {
      continue;
    }

    if (!S_ISCHR(stat_buf.st_mode)) {  // input devices are character devices, skip other
      continue;
    }

    // get joystick name
    int joy_fd = open(current_path.c_str(), O_RDONLY);

    if (joy_fd == -1) {
      continue;
    }

    char current_joy_name[128];

    if (ioctl(joy_fd, JSIOCGNAME(sizeof(current_joy_name)), current_joy_name) < 0) {
      std::strncpy(current_joy_name, "Unknown", sizeof(current_joy_name));
    }

    close(joy_fd);

    RCLCPP_INFO(
      this->get_logger(),
      "Found joystick: %s (%s).",
      current_joy_name,
      current_path.c_str());

    if (std::strcmp(current_joy_name, name.c_str()) == 0) {
      closedir(dev_dir);
      return current_path;
    }
  }

  closedir(dev_dir);
  return "";
}

}  // namespace joy

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(joy::JoyNode)
