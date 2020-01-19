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

#ifndef JOY__JOY_NODE_HPP_
#define JOY__JOY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>

#include <memory>
#include <string>

namespace joy
{
class JoyNode
  : public rclcpp::Node
{
public:
  explicit JoyNode(rclcpp::NodeOptions options);

private:
  std::string dev_path;
  std::string dev_ff_path;
  std::string dev_name;
  double deadzone;
  double autorepeat_rate;
  double coalesce_interval;
  bool sticky_buttons;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Joy>> joy_pub;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JoyFeedbackArray>> feedback_sub;

  void feedback_callback(const sensor_msgs::msg::JoyFeedbackArray::SharedPtr msg);
  std::string get_dev_path_by_name(const std::string name);
};
}  // namespace joy

#endif  // JOY__JOY_NODE_HPP_
