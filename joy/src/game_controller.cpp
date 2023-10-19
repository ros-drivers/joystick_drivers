/*
 * Copyright (c) 2020, Open Source Robotics Foundation.
 * Copyright (c) 2023, CSIRO Data61.
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

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <SDL.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>

#include "joy/game_controller.hpp"

namespace joy
{

GameController::GameController(const rclcpp::NodeOptions & options)
: rclcpp::Node("game_controller_node", options)
{
  dev_id_ = static_cast<int>(this->declare_parameter("device_id", 0));

  dev_name_ = this->declare_parameter("device_name", std::string(""));

  // The user specifies the deadzone to us in the range of 0.0 to 1.0.  Later on
  // we'll convert that to the range of 0 to 32767.  Note also that negatives
  // are not allowed, as this is a +/- value.
  scaled_deadzone_ = this->declare_parameter("deadzone", 0.05);
  if (scaled_deadzone_ < 0.0 || scaled_deadzone_ > 1.0) {
    throw std::runtime_error("Deadzone must be between 0.0 and 1.0");
  }
  unscaled_deadzone_ = 32767.0 * scaled_deadzone_;
  // According to the SDL documentation, this always returns a value between
  // -32768 and 32767.  However, we want to report a value between -1.0 and 1.0,
  // hence the "scale" dividing by 32767.  Also note that SDL returns the axes
  // with "forward" and "left" as negative.  This is opposite to the ROS
  // conventionof "forward" and "left" as positive, so we invert the axes here
  // as well.  Finally, we take into account the amount of deadzone so we truly
  // do get value between -1.0 and 1.0 (and not -deadzone to +deadzone).
  scale_ = static_cast<float>(-1.0 / (1.0 - scaled_deadzone_) / 32767.0);

  autorepeat_rate_ = this->declare_parameter("autorepeat_rate", 20.0);
  if (autorepeat_rate_ < 0.0) {
    throw std::runtime_error("Autorepeat rate must be >= 0.0");
  } else if (autorepeat_rate_ > 1000.0) {
    throw std::runtime_error("Autorepeat rate must be <= 1000.0");
  } else if (autorepeat_rate_ > 0.0) {
    autorepeat_interval_ms_ = static_cast<int>(1000.0 / autorepeat_rate_);
  } else {
    // If the autorepeat rate is set to 0, the user doesn't want us to
    // publish unless an event happens.  We still wake up every 200
    // milliseconds to check if we need to quit.
    autorepeat_interval_ms_ = 200;
  }

  sticky_buttons_ = this->declare_parameter("sticky_buttons", false);

  coalesce_interval_ms_ = static_cast<int>(this->declare_parameter("coalesce_interval_ms", 1));
  if (coalesce_interval_ms_ < 0) {
    throw std::runtime_error("coalesce_interval_ms must be positive");
  }
  // Make sure to initialize publish_soon_time regardless of whether we are going
  // to use it; this ensures that we are always using the correct time source.
  publish_soon_time_ = this->now();

  pub_ = create_publisher<sensor_msgs::msg::Joy>("joy", 10);

  feedback_sub_ = this->create_subscription<sensor_msgs::msg::JoyFeedback>(
    "joy/set_feedback", rclcpp::QoS(10),
    std::bind(&GameController::feedbackCb, this, std::placeholders::_1));

  future_ = exit_signal_.get_future();

  // In theory we could do this with just a timer, which would simplify the code
  // a bit.  But then we couldn't react to "immediate" events, so we stick with
  // the thread.
  event_thread_ = std::thread(&GameController::eventThread, this);

  joy_msg_.buttons.resize(SDL_CONTROLLER_BUTTON_MAX);

  joy_msg_.axes.resize(SDL_CONTROLLER_AXIS_MAX);

  if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
    throw std::runtime_error("SDL could not be initialized: " + std::string(SDL_GetError()));
  }
}

GameController::~GameController()
{
  exit_signal_.set_value();
  event_thread_.join();
  if (game_controller_ != nullptr) {
    SDL_GameControllerClose(game_controller_);
  }
  SDL_Quit();
}

void GameController::feedbackCb(const std::shared_ptr<sensor_msgs::msg::JoyFeedback> msg)
{
  if (msg->type != sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE) {
    // We only support rumble
    return;
  }

  if (msg->id != 0) {
    // There can be only one (rumble)
    // TODO(Rod Taylor): Support high and low frequency rumble channels.
    return;
  }

  if (msg->intensity < 0.0 || msg->intensity > 1.0) {
    // We only accept intensities between 0 and 1.
    return;
  }

  if (game_controller_ != nullptr) {
    // We purposely ignore the return value; if it fails, what can we do?
    uint16_t intensity = static_cast<uint16_t>(msg->intensity * 0xFFFF);
    SDL_GameControllerRumble(game_controller_, intensity, intensity, 1000);
  }
}

float GameController::convertRawAxisValueToROS(int16_t val)
{
  // SDL reports axis values between -32768 and 32767.  To make sure
  // we report out scaled value between -1.0 and 1.0, we add one to
  // the value iff it is exactly -32768.  This makes all of the math
  // below work properly.
  if (val == -32768) {
    val = -32767;
  }

  // Note that we do all of the math in double space below.  This ensures
  // that the values stay between -1.0 and 1.0.
  double double_val = static_cast<double>(val);
  // Apply the deadzone semantic here.  This allows the deadzone
  // to be "smooth".
  if (double_val > unscaled_deadzone_) {
    double_val -= unscaled_deadzone_;
  } else if (double_val < -unscaled_deadzone_) {
    double_val += unscaled_deadzone_;
  } else {
    double_val = 0.0;
  }

  return static_cast<float>(double_val * scale_);
}

bool GameController::handleControllerAxis(const SDL_ControllerAxisEvent & e)
{
  bool publish = false;

  if (e.which != joystick_instance_id_) {
    return publish;
  }

  if (e.axis >= SDL_CONTROLLER_AXIS_MAX) {
    RCLCPP_WARN(get_logger(), "Saw axes too large for this device, ignoring");
    return publish;
  }

  float last_axis_value = joy_msg_.axes.at(e.axis);
  joy_msg_.axes.at(e.axis) = convertRawAxisValueToROS(e.value);
  if (last_axis_value != joy_msg_.axes.at(e.axis)) {
    if (coalesce_interval_ms_ > 0 && !publish_soon_) {
      publish_soon_ = true;
      publish_soon_time_ = this->now();
    } else {
      rclcpp::Duration time_since_publish_soon = this->now() - publish_soon_time_;
      if (time_since_publish_soon.nanoseconds() >= RCL_MS_TO_NS(coalesce_interval_ms_)) {
        publish = true;
        publish_soon_ = false;
      }
    }
  }
  // else no change, so don't publish

  return publish;
}

bool GameController::handleControllerButtonDown(const SDL_ControllerButtonEvent & e)
{
  bool publish = false;

  if (e.which != joystick_instance_id_) {
    return publish;
  }

  if (e.button >= SDL_CONTROLLER_BUTTON_MAX) {
    RCLCPP_WARN(get_logger(), "Saw button too large for this device, ignoring");
    return publish;
  }

  if (sticky_buttons_) {
    // For sticky buttons, invert 0 -> 1 or 1 -> 0
    joy_msg_.buttons.at(e.button) = 1 - joy_msg_.buttons.at(e.button);
  } else {
    joy_msg_.buttons.at(e.button) = 1;
  }
  publish = true;

  return publish;
}

bool GameController::handleControllerButtonUp(const SDL_ControllerButtonEvent & e)
{
  bool publish = false;

  if (e.which != joystick_instance_id_) {
    return publish;
  }

  if (e.button >= SDL_CONTROLLER_BUTTON_MAX) {
    RCLCPP_WARN(get_logger(), "Saw button too large for this device, ignoring");
    return publish;
  }

  if (!sticky_buttons_) {
    joy_msg_.buttons.at(e.button) = 0;
    publish = true;
  }
  return publish;
}

void GameController::handleControllerDeviceAdded(const SDL_ControllerDeviceEvent & e)
{
  int num_joysticks = SDL_NumJoysticks();
  if (num_joysticks < 0) {
    RCLCPP_WARN(get_logger(), "Failed to get the number of game controllers: %s", SDL_GetError());
    return;
  }
  bool matching_device_found = false;
  for (int i = 0; i < num_joysticks; ++i) {
    const char * name = SDL_JoystickNameForIndex(i);
    if (name == nullptr) {
      RCLCPP_WARN(get_logger(), "Could not get game controller name: %s", SDL_GetError());
      continue;
    }
    RCLCPP_INFO(get_logger(), "Controller Found: device_id=%i, device_name=%s", i, name);
    if (std::string(name) == dev_name_) {
      if (!dev_name_.empty()) {
        // We found it!
        matching_device_found = true;
        dev_id_ = i;
        break;
      }
    }
  }
  if (!dev_name_.empty() && !matching_device_found) {
    RCLCPP_WARN(
      get_logger(), "Could not get game controller with name %s: %s",
      dev_name_.c_str(), SDL_GetError());
    return;
  }

  if (e.which != dev_id_) {
    // ignore device that don't match the dev_id_ specified
    return;
  }

  game_controller_ = SDL_GameControllerOpen(dev_id_);
  if (game_controller_ == nullptr) {
    RCLCPP_WARN(get_logger(), "Unable to open game controller %d: %s", dev_id_, SDL_GetError());
    return;
  }

  // We need to hold onto this so that we can properly remove it on a
  // remove event.
  joystick_instance_id_ = SDL_JoystickGetDeviceInstanceID(dev_id_);
  if (joystick_instance_id_ < 0) {
    RCLCPP_WARN(get_logger(), "Failed to get instance ID for game controller: %s", SDL_GetError());
    SDL_GameControllerClose(game_controller_);
    game_controller_ = nullptr;
    return;
  }

  // Get the initial state for each of the axes
  for (int i = 0; i < SDL_CONTROLLER_AXIS_MAX; ++i) {
    int16_t state =
      SDL_GameControllerGetAxis(game_controller_, static_cast<SDL_GameControllerAxis>(i));
    joy_msg_.axes.at(i) = convertRawAxisValueToROS(state);
  }

#if SDL_VERSION_ATLEAST(2, 0, 18)
  const char * has_rumble_string = "No";
  if (SDL_GameControllerHasRumble(game_controller_)) {
    has_rumble_string = "Yes";
  }
#else
  const char * has_rumble_string = "Unknown";
#endif

  RCLCPP_INFO(
    get_logger(), "Opened game controller: %s,  deadzone: %f, rumble: %s",
    SDL_GameControllerName(game_controller_), scaled_deadzone_, has_rumble_string);
}

void GameController::handleControllerDeviceRemoved(const SDL_ControllerDeviceEvent & e)
{
  if (e.which != joystick_instance_id_) {
    return;
  }
  if (game_controller_ != nullptr) {
    RCLCPP_INFO(
      get_logger(), "Game controller Removed: %s.",
      SDL_GameControllerName(game_controller_));
    SDL_GameControllerClose(game_controller_);
    game_controller_ = nullptr;
  }
}

void GameController::eventThread()
{
  std::future_status status;
  rclcpp::Time last_publish = this->now();

  do{
    bool should_publish = false;
    SDL_Event e;
    int wait_time_ms = autorepeat_interval_ms_;
    if (publish_soon_) {
      wait_time_ms = std::min(wait_time_ms, coalesce_interval_ms_);
    }
    int success = SDL_WaitEventTimeout(&e, wait_time_ms);
    if (success == 1) {
      // Succeeded getting an event
      switch (e.type) {
        case SDL_CONTROLLERAXISMOTION: {
            should_publish = handleControllerAxis(e.caxis);
            break;
          }
        case SDL_CONTROLLERBUTTONDOWN: {
            should_publish = handleControllerButtonDown(e.cbutton);
            break;
          }
        case SDL_CONTROLLERBUTTONUP: {
            should_publish = handleControllerButtonUp(e.cbutton);
            break;
          }
        case SDL_CONTROLLERDEVICEADDED: {
            handleControllerDeviceAdded(e.cdevice);
            break;
          }
        case SDL_CONTROLLERDEVICEREMOVED: {
            handleControllerDeviceRemoved(e.cdevice);
            break;
          }
        case SDL_JOYAXISMOTION:  // Ignore joystick events, they are duplicates of CONTROLLERDEVICE.
        case SDL_JOYBALLMOTION:
        case SDL_JOYHATMOTION:
        case SDL_JOYBUTTONDOWN:
        case SDL_JOYBUTTONUP:
        case SDL_JOYDEVICEADDED:
        case SDL_JOYDEVICEREMOVED: {
            break;
          }
        default: {
            RCLCPP_INFO(get_logger(), "Unknown event type %d", e.type);
            break;
          }
      }
    } else {
      // We didn't succeed, either because of a failure or because of a timeout.
      // If we are autorepeating and enough time has passed, set should_publish.
      rclcpp::Time now = this->now();
      rclcpp::Duration diff_since_last_publish = now - last_publish;
      if ((autorepeat_rate_ > 0.0 &&
        RCL_NS_TO_MS(diff_since_last_publish.nanoseconds()) >= autorepeat_interval_ms_) ||
        publish_soon_)
      {
        last_publish = now;
        should_publish = true;
        publish_soon_ = false;
      }
    }

    if (game_controller_ != nullptr && should_publish) {
      joy_msg_.header.frame_id = "joy";
      joy_msg_.header.stamp = this->now();

      pub_->publish(joy_msg_);
    }

    status = future_.wait_for(std::chrono::seconds(0));
  } while (status == std::future_status::timeout);
}

}  // namespace joy

RCLCPP_COMPONENTS_REGISTER_NODE(joy::GameController)
