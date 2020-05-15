/*
 * Copyright (c) 2020, Bundesanstalt für Materialforschung und -prüfung (BAM).
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
 *     * Neither the name of the copyright holder nor the names of its
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

#include <joy_linux/configuration.hpp>
#include <joy_linux/enumeration.hpp>

#include <algorithm>

JoystickConfiguration loadConfiguration(std::shared_ptr<rclcpp::Node> node)
{
  static constexpr char DEFAULT_DEVICE[] = "/dev/input/js0";

  JoystickConfiguration config;

  config.device = node->declare_parameter("dev", std::string(""));

  if (config.device.empty()) {
    // if no device path was specified, check for dev_name
    std::string device_name = node->declare_parameter("dev_name", std::string(""));
    if (device_name.empty()) {
      config.device = DEFAULT_DEVICE;
      RCLCPP_WARN_STREAM(node->get_logger(), "Neither dev nor dev_name were specified."
                                             " Will use default device: " << config.device);
    } else {
      // enumerate all joysticks and check if one matches device_name
      RCLCPP_INFO_STREAM(node->get_logger(), "Looking for joystick with name: " << device_name);
      std::vector<JoystickData> joysticks = getJoysticks(node->get_logger());

      auto joystick_it = std::find_if(joysticks.begin(), joysticks.end(),
        [&device_name](const JoystickData &data) {
          return data.device_name == device_name; });

      if (joystick_it != joysticks.end()) { // found a joystick matching device_name
        config.device = joystick_it->device_path;
        RCLCPP_INFO_STREAM(node->get_logger(), "Will use device: " << config.device);
      } else {
        config.device = DEFAULT_DEVICE;
        RCLCPP_WARN_STREAM(node->get_logger(), "Joystick with name " << device_name
                           << " not found, will use default device: " << config.device);
      }
    }
  }

  config.feedback_device = node->declare_parameter("dev_ff", std::string(""));
  config.deadzone = node->declare_parameter("deadzone", 0.05);
  config.autorepeat_rate = node->declare_parameter("autorepeat_rate", 20.0);
  config.coalesce_interval = node->declare_parameter("coalesce_interval", 0.001);
  config.default_trig_val = node->declare_parameter("default_trig_val", false);
  config.sticky_buttons = node->declare_parameter("sticky_buttons", false);

  // Checks on parameters
  if (config.autorepeat_rate > 1 / config.coalesce_interval) {
    RCLCPP_WARN(
      node->get_logger(), "joy_linux_node: autorepeat_rate (%f Hz) > "
      "1/coalesce_interval (%f Hz) does not make sense. Timing behavior is not well defined.",
      config.autorepeat_rate, 1 / config.coalesce_interval);
  }

  if (config.deadzone >= 1) {
    RCLCPP_WARN(
      node->get_logger(), "joy_linux_node: deadzone greater than 1 was requested. "
      "The semantics of deadzone have changed. It is now related to the range [-1:1] instead "
      "of [-32767:32767]. For now I am dividing your deadzone by 32767, but this behavior is "
      "deprecated so you need to update your launch file.");
    config.deadzone /= 32767;
  }

  if (config.deadzone > 0.9) {
    RCLCPP_WARN(
      node->get_logger(), "joy_node: deadzone (%f) greater than 0.9, setting it to 0.9",
      config.deadzone);
    config.deadzone = 0.9;
  }

  if (config.deadzone < 0) {
    RCLCPP_WARN(
      node->get_logger(), "joy_node: deadzone_ (%f) less than 0, setting to 0.", config.deadzone);
    config.deadzone = 0;
  }

  if (config.autorepeat_rate < 0) {
    RCLCPP_WARN(
      node->get_logger(), "joy_node: autorepeat_rate (%f) less than 0, setting to 0.",
      config.autorepeat_rate);
    config.autorepeat_rate = 0;
  }

  if (config.coalesce_interval < 0) {
    RCLCPP_WARN(
      node->get_logger(), "joy_node: coalesce_interval (%f) less than 0, setting to 0.",
      config.coalesce_interval);
    config.coalesce_interval = 0;
  }

  // Parameter conversions
  config.scale = -1. / (1. - config.deadzone) / 32767.;
  config.unscaled_deadzone = 32767. * config.deadzone;

  return config;
}
