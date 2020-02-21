#ifndef ROS2_JOY_JOY_NODE_HPP_INCLUDED
#define ROS2_JOY_JOY_NODE_HPP_INCLUDED

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <string>

#include "joystick_data.hpp"

enum class ProcessEventsResult { Continue, Abort, PublishNow, PublishSoon };

class JoyNode : public rclcpp::Node
{
public:
  JoyNode(rclcpp::NodeOptions options);
  virtual ~JoyNode();

  void run();

protected:
  virtual bool tryOpen(JoystickData &joystick_data) = 0; // Opens the joystick. Returns true on success, false otherwise.
  virtual void close() = 0;
  virtual ProcessEventsResult processEvents() = 0;

  virtual void handleSetFeedback(const sensor_msgs::msg::JoyFeedbackArray::SharedPtr msg) = 0;

  void startAutorepeatPublishing(); // starts the autorepeat timer
  void updateButton(size_t button, int32_t value);
  void updateAxis(size_t axis, float value);

private:
  void publish();
  void handleCoalesceTimerEvent();

  double deadzone;
  double deadzone_scale;
  double unscaled_deadzone;
  double autorepeat_rate; // [Hz], 0 for no repeat
  double coalesce_interval; // [s], 0 for no coalescence (changes are published immediately)
  bool sticky_buttons;

  // publishing
  rclcpp::Clock joy_clock;
  sensor_msgs::msg::Joy joy_msg;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Joy>> joy_pub;
  std::shared_ptr<rclcpp::TimerBase> joy_pub_coalesce_timer;
  std::shared_ptr<rclcpp::TimerBase> joy_pub_autorepeat_timer;

  // subscriber
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JoyFeedbackArray>> feedback_sub;
};

#endif // ROS2_JOY_JOY_NODE_HPP_INCLUDED
