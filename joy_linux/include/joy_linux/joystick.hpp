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

#ifndef JOY_LINUX__JOYSTICK_HPP_
#define JOY_LINUX__JOYSTICK_HPP_

#include <joy_linux/configuration.hpp>
#include <joy_linux/feedback.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <list>
#include <memory>

class Joystick
{
public:
  Joystick();

  int run();

private:
  bool open(bool first_fault);
  void close();
  bool handleEvents(double max_wait_time);  // returns false on I/O error
  void processJoystickEvent(const js_event &event);
  void checkInitEvents();
  void updateButton(size_t button, int32_t value);
  void updateAxis(size_t axis, float value);
  void publish();

  std::shared_ptr<rclcpp::Node> node_;
  JoystickConfiguration config_;
  int fd_;

  FeedbackDevice feedback_device_;

  // initial events
  std::list<unsigned> pending_axis_init_events_;
  std::list<unsigned> pending_button_init_events_;
  bool initialisation_done_;

  // publishing
  sensor_msgs::msg::Joy joy_msg_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Joy>> joy_pub_;
  bool publish_now_;
  bool publish_soon_;
};

#endif  // JOY_LINUX__JOYSTICK_HPP_
