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

#include <joy_linux/joystick.hpp>
#include <joy_linux/enumeration.hpp>

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

Joystick::Joystick()
: node_(std::make_shared<rclcpp::Node>("joy_node")),
  config_(loadConfiguration(node_)),
  fd_(-1),
  feedback_device_(node_, config_.feedback_device),
  joy_pub_(node_->create_publisher<sensor_msgs::msg::Joy>("joy", 10))
{
  joy_msg_.header.frame_id = "joy";
}

bool Joystick::open(bool first_fault)
{
  // There seems to be a bug in the driver or something where the
  // initial events that are to define the initial state of the
  // joystick are not the values of the joystick when it was opened
  // but rather the values of the joystick when it was last closed.
  // Opening then closing and opening again is a hack to get more
  // accurate initial state data.
  int temp_fd = ::open(config_.device.c_str(), O_RDONLY);
  if (temp_fd == -1) {
    if (first_fault) {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(), "Cannot open " << config_.device <<
          ". Error " << errno << ": " << strerror(errno));
    }
    return false;
  }
  ::close(temp_fd);

  // fillJoystickData will open and close the device
  JoystickData joystick_data;
  if (!fillJoystickData(config_.device, joystick_data, node_->get_logger())) {
    return false;
  }

  // Prepare joy message and init events for the specific device
  initialisation_done_ = false;
  pending_axis_init_events_.clear();
  pending_button_init_events_.clear();
  joy_msg_.axes.clear();
  joy_msg_.buttons.clear();

  for (size_t i = 0; i < joystick_data.number_of_axes; ++i) {
    pending_axis_init_events_.emplace_back(i);
    joy_msg_.axes.emplace_back(0);
  }

  for (size_t i = 0; i < joystick_data.number_of_buttons; ++i) {
    pending_button_init_events_.emplace_back(i);
    joy_msg_.buttons.emplace_back(0);
  }

  fd_ = ::open(config_.device.c_str(), O_RDONLY);
  if (fd_ == -1) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(), "Cannot open " << config_.device <<
        ". Error " << errno << ": " << strerror(errno));
    return false;
  }

  RCLCPP_INFO_STREAM(
    node_->get_logger(), "Opened joystick: " << config_.device <<
      " (" << joystick_data.device_name << "). deadzone: " << config_.deadzone);

  return true;
}

void Joystick::close()
{
  if (fd_ != -1) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool Joystick::handleEvents(double max_wait_time)
{
  fd_set set;
  FD_ZERO(&set);
  FD_SET(fd_, &set);

  max_wait_time = std::max(0.0, max_wait_time);
  struct timeval timeout;
  timeout.tv_sec = trunc(max_wait_time);
  timeout.tv_usec = (max_wait_time - timeout.tv_sec) * 1e6;

  int select_result = ::select(fd_ + 1, &set, nullptr, nullptr, &timeout);
  if (select_result > 0 && FD_ISSET(fd_, &set)) {  // the file descriptor has data to read
    js_event event;
    if (::read(fd_, &event, sizeof(event)) == -1) {
      if (errno == EAGAIN) {  // read would block
        return true;  // try again
      } else if (errno == EINTR) {  // read was interrupted by a signal
        return false;
      }
      RCLCPP_ERROR_STREAM(
        node_->get_logger(), "Error " << errno << " while reading from joystick device: " <<
          strerror(errno));
      return false;
    }

    processJoystickEvent(event);
  } else if (select_result == 0) {  // a timeout occured
    return true;  // try again
  } else if (select_result < 0) {  // an error occured
    if (errno == EINTR) {  // select was interrupted by a signal
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(), "Error " << errno << " while waiting for joystick device: " <<
        strerror(errno));
    return false;
  }

  return true;
}

void Joystick::processJoystickEvent(const js_event & event)
{
  switch (event.type) {
    case JS_EVENT_AXIS:
      updateAxis(event.number, event.value);
      break;
    case JS_EVENT_BUTTON:
      updateButton(event.number, event.value);
      break;
    case JS_EVENT_AXIS | JS_EVENT_INIT:
      updateAxis(event.number, event.value);
      pending_axis_init_events_.remove(event.number);
      checkInitEvents();
      break;
    case JS_EVENT_BUTTON | JS_EVENT_INIT:
      updateButton(event.number, event.value);
      pending_button_init_events_.remove(event.number);
      checkInitEvents();
      break;
    default:
      RCLCPP_WARN(
        node_->get_logger(), "joy_linux_node: Unknown event type. "
        "Please file a ticket. time=%u, value=%d, type=%Xh, number=%d",
        event.time, event.value, event.type, event.number);
      break;
  }
}

void Joystick::updateButton(size_t button, int32_t value)
{
  if (config_.sticky_buttons) {
    if (value == 1) {
      joy_msg_.buttons.at(button) = 1 - joy_msg_.buttons.at(button);
    }
  } else {
    joy_msg_.buttons.at(button) = value ? 1 : 0;
  }

  publish_now_ = true;
}

void Joystick::updateAxis(size_t axis, float value)
{
  // Allows deadzone to be "smooth"
  if (value > config_.unscaled_deadzone) {
    value -= config_.unscaled_deadzone;
  } else if (value < -config_.unscaled_deadzone) {
    value += config_.unscaled_deadzone;
  } else {
    value = 0;
  }
  joy_msg_.axes.at(axis) = value * config_.scale;

  // Will wait a bit before sending to try to combine events.
  publish_soon_ = true;
}

void Joystick::checkInitEvents()
{
  if (pending_axis_init_events_.empty() && pending_button_init_events_.empty()) {
    // RCLCPP_INFO(node_->get_logger(), "All init events received");
    initialisation_done_ = true;
    publish_now_ = true;
  }
}

void Joystick::publish()
{
  joy_msg_.header.stamp = node_->now();
  joy_pub_->publish(joy_msg_);
}

int Joystick::run()
{
  rclcpp::executors::SingleThreadedExecutor ros_executor;

  while (rclcpp::ok()) {
    bool first_fault = true;
    while (true) {
      // In the first iteration of this loop, first_fault is true so we just
      // want to check for rclcpp work and not block.  If it turns out that
      // we cannot open the joystick device immediately, then in subsequent
      // iterations we block for up to a second in rclcpp before attempting
      // to open the joystick device again.  The dummy promise and future
      // are used to accomplish this 1 second wait.
      std::promise<void> dummy_promise;
      std::shared_future<void> dummy_future(dummy_promise.get_future());
      std::chrono::duration<int64_t, std::milli> timeout;
      if (first_fault) {
        timeout = std::chrono::milliseconds(0);
      } else {
        timeout = std::chrono::milliseconds(1000);
      }
      rclcpp::spin_until_future_complete(node_, dummy_future, timeout);
      if (!rclcpp::ok()) {
        goto cleanup;
      }
      if (open(first_fault)) {
        break;
      }
      if (first_fault) {
        RCLCPP_ERROR_STREAM(
          node_->get_logger(), "Couldn't open joystick " << config_.device <<
            ". Will retry every second.");
        first_fault = false;
      }
    }

    feedback_device_.open();

    constexpr auto NO_NEXT = std::chrono::steady_clock::time_point::min();
    std::chrono::steady_clock::time_point t_publish_next = NO_NEXT;

    while (rclcpp::ok()) {
      ros_executor.spin_node_some(node_);

      publish_now_ = false;
      publish_soon_ = false;

      // play the rumble effect (can probably do this at lower rate later)
      feedback_device_.uploadEffect();

      double max_wait_time = 1.0;  // [s]
      if (t_publish_next != NO_NEXT) {
        max_wait_time = std::chrono::duration_cast<std::chrono::microseconds>(
          t_publish_next - std::chrono::steady_clock::now()).count() * 1e-6;
      }

      if (!handleEvents(max_wait_time)) {
        break;
      }

      // If an axis event occurred, start a timer to combine with other events.
      if (publish_soon_) {
        std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now() +
          std::chrono::microseconds(static_cast<int>(config_.coalesce_interval * 1e6));
        if (t < t_publish_next || t_publish_next == NO_NEXT) {
          t_publish_next = t;
        }
      }

      if (std::chrono::steady_clock::now() >= t_publish_next && t_publish_next != NO_NEXT) {
        publish_now_ = true;
      }

      if (publish_now_) {
        if (initialisation_done_ || config_.default_trig_val) {
          publish();
        }

        // do autorepeat (if nothing else will happen)
        if (config_.autorepeat_rate > 0) {
          double autorepeat_interval = 1 / config_.autorepeat_rate;
          t_publish_next = std::chrono::steady_clock::now() +
            std::chrono::microseconds(static_cast<int>(autorepeat_interval * 1e6));
        } else {
          t_publish_next = NO_NEXT;
        }
      }
    }  // End of joystick open loop.

    feedback_device_.close();
    close();
    ros_executor.spin_node_some(node_);
    if (rclcpp::ok()) {
      RCLCPP_ERROR(
        node_->get_logger(), "Connection to joystick device lost unexpectedly. Will reopen.");
    }
  }

cleanup:
  RCLCPP_INFO(node_->get_logger(), "joy_node shut down.");

  return 0;
}
