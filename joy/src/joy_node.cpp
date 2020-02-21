/*
 * teleop_pr2
 * Copyright (c) 2009, Willow Garage, Inc.
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

//\author: Blaise Gassend

#include <joy/linux_joystick.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  try {
    auto joy_node = std::make_shared<LinuxJoystick>(rclcpp::NodeOptions());
    joy_node->run();
  }
  catch (const std::exception &e) {
    //
  }

  return rclcpp::shutdown() ? 0 : -1;
}

JoyNode::JoyNode(rclcpp::NodeOptions options)
: Node("joy_node", options),
  joy_pub(this->create_publisher<sensor_msgs::msg::Joy>("joy", 1)),
  feedback_sub(this->create_subscription<sensor_msgs::msg::JoyFeedbackArray>(
    "joy/set_feedback", 10, std::bind(&JoyNode::handleSetFeedback, this, std::placeholders::_1)))
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Starting node " << this->get_name());

  deadzone = this->declare_parameter("deadzone", 0.05);
  autorepeat_rate = this->declare_parameter("autorepeat_rate", 0.0);
  coalesce_interval = this->declare_parameter("coalesce_interval", 0.001);
  // TODO(jwhitleyastuff): Make "default_trig_val actually do what we expect.
  sticky_buttons = this->declare_parameter("sticky_buttons", false);

  if (autorepeat_rate > (1.0 / coalesce_interval)) {
    RCLCPP_WARN(this->get_logger(),
      "autorepeat_rate (%f Hz) > 1 / coalesce_interval (%f Hz) does not make sense. "
      "Timing behavior is not well defined.",
      autorepeat_rate, 1.0 / coalesce_interval);
  }

  if (deadzone >= 1) {
    RCLCPP_WARN(this->get_logger(), "deadzone (%f) greater than 1.0, setting to 0.9.", deadzone);
    deadzone = 0.9;
  } else if (deadzone > 0.9) {
    RCLCPP_WARN(this->get_logger(), "deadzone (%f) greater than 0.9, setting to 0.9.", deadzone);
    deadzone = 0.9;
  } else if (deadzone < 0) {
    RCLCPP_WARN(this->get_logger(), "deadzone (%f) less than 0, setting to 0.", deadzone);
    deadzone = 0;
  }
  deadzone_scale = -1.0 / (1.0 - deadzone) / 32767.0;
  unscaled_deadzone = 32767.0 * deadzone;

  if (autorepeat_rate < 0) {
    RCLCPP_WARN(this->get_logger(), "autorepeat_rate (%f) less than 0, setting to 0.", autorepeat_rate);
    autorepeat_rate = 0; // 0 --> disable autorepeat
  } else if (autorepeat_rate > 0) {
    double autorepeat_interval = 1.0 / autorepeat_rate;
    joy_pub_autorepeat_timer = this->create_wall_timer(std::chrono::microseconds(static_cast<long>(autorepeat_interval * 1e6)),
                                                       [this]{ publish(); });
    joy_pub_autorepeat_timer->cancel();
  }

  if (coalesce_interval < 0) {
    RCLCPP_WARN(this->get_logger(), "coalesce_interval (%f) less than 0, setting to 0.", coalesce_interval);
    coalesce_interval = 0; // 0 --> publish immediately
  } else if (coalesce_interval > 0) {
    joy_pub_coalesce_timer = this->create_wall_timer(std::chrono::microseconds(static_cast<long>(coalesce_interval * 1e6)),
                                                     [this]{ handleCoalesceTimerEvent(); });
    joy_pub_coalesce_timer->cancel();
  }
}

JoyNode::~JoyNode()
{
  RCLCPP_INFO(this->get_logger(), "Exiting node");
}

void JoyNode::run()
{
  while (rclcpp::ok()) {
    // Try to open the joystick
    bool opened = false;
    JoystickData joystick_data;
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->shared_from_this());
      opened = tryOpen(joystick_data);
      if (opened) break;
      RCLCPP_ERROR_STREAM_ONCE(this->get_logger(),
        "Could not open joystick. Will retry every second.");
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    if (!opened) break;
    RCLCPP_INFO_STREAM(this->get_logger(), "Opened joystick " << joystick_data.toBasicInfoString()
                       << ". Deadzone: " << deadzone
                       << " Autorepeat rate: " << autorepeat_rate
                       << " Coalesce interval: " << coalesce_interval);

    // Prepare joy message for the specific device
    joy_msg.axes.clear();
    joy_msg.buttons.clear();
    for (size_t i = 0; i < joystick_data.number_of_axes; ++i)
      joy_msg.axes.emplace_back(0);
    for (size_t i = 0; i < joystick_data.number_of_buttons; ++i)
      joy_msg.buttons.emplace_back(0);

    // Handle joystick events
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->shared_from_this());
      ProcessEventsResult result = processEvents();
      if (result == ProcessEventsResult::Continue) {
        continue;
      } else if (result == ProcessEventsResult::PublishNow) {
        publish();
      } else if (result == ProcessEventsResult::PublishSoon) {
        if (!joy_pub_coalesce_timer) {
          publish();
          if (joy_pub_autorepeat_timer) joy_pub_autorepeat_timer->reset();
        }
        else if (joy_pub_coalesce_timer->is_canceled()) {
          if (joy_pub_autorepeat_timer) joy_pub_autorepeat_timer->cancel();
          joy_pub_coalesce_timer->reset();
        }
      } else if (result == ProcessEventsResult::Abort)
        break;
    }

    if (joy_pub_coalesce_timer) joy_pub_coalesce_timer->cancel();
    if (joy_pub_autorepeat_timer) joy_pub_autorepeat_timer->cancel();
    close();
  }
}

void JoyNode::startAutorepeatPublishing()
{
  if (joy_pub_autorepeat_timer) {
    RCLCPP_INFO(this->get_logger(), "Starting autorepeat publishing");
    joy_pub_autorepeat_timer->reset();
  }
}

void JoyNode::updateButton(size_t button, int32_t value)
{
  joy_msg.buttons.at(button) = value;
}

void JoyNode::updateAxis(size_t axis, float value)
{
  joy_msg.axes.at(axis) = value;
}

void JoyNode::publish()
{
  joy_msg.header.stamp = joy_clock.now();
  joy_pub->publish(joy_msg);
}

void JoyNode::handleCoalesceTimerEvent()
{
  joy_pub_coalesce_timer->cancel();
  if (joy_pub_autorepeat_timer) joy_pub_autorepeat_timer->reset();
  publish();
}
