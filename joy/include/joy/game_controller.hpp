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

#ifndef JOY__GAME_CONTROLLER_HPP_
#define JOY__GAME_CONTROLLER_HPP_

#include <SDL.h>

#include <future>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>

namespace joy
{

class GameController final : public rclcpp::Node
{
public:
  explicit GameController(const rclcpp::NodeOptions & options);
  GameController(GameController && c) = delete;
  GameController & operator=(GameController && c) = delete;
  GameController(const GameController & c) = delete;
  GameController & operator=(const GameController & c) = delete;

  ~GameController() override;

private:
  void eventThread();
  bool handleControllerAxis(const SDL_ControllerAxisEvent & e);
  bool handleControllerButtonDown(const SDL_ControllerButtonEvent & e);
  bool handleControllerButtonUp(const SDL_ControllerButtonEvent & e);
  void handleControllerDeviceAdded(const SDL_ControllerDeviceEvent & e);
  void handleControllerDeviceRemoved(const SDL_ControllerDeviceEvent & e);
  float convertRawAxisValueToROS(int16_t val);
  void feedbackCb(const std::shared_ptr<sensor_msgs::msg::JoyFeedback> msg);

  int dev_id_{0};

  SDL_GameController * game_controller_{nullptr};
  SDL_JoystickID joystick_instance_id_{0};
  double scaled_deadzone_{0.0};
  double unscaled_deadzone_{0.0};
  double scale_{0.0};
  double autorepeat_rate_{0.0};
  int autorepeat_interval_ms_{0};
  bool sticky_buttons_{false};
  bool publish_soon_{false};
  rclcpp::Time publish_soon_time_;
  int coalesce_interval_ms_{0};
  std::string dev_name_;
  std::thread event_thread_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JoyFeedback>::SharedPtr feedback_sub_;

  sensor_msgs::msg::Joy joy_msg_;
};

}  // namespace joy

#endif  // JOY__GAME_CONTROLLER_HPP_
