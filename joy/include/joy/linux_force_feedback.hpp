#ifndef ROS2_JOY_LINUX_FORCE_FEEDBACK_HPP_INCLUDED
#define ROS2_JOY_LINUX_FORCE_FEEDBACK_HPP_INCLUDED

#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <string>

struct ff_effect;

class LinuxForceFeedbackDevice
{
public:
  LinuxForceFeedbackDevice(const std::string &device_path, rclcpp::Logger parent_logger);
  ~LinuxForceFeedbackDevice();

  void open();
  void close();
  bool isOpen() const;

  void handleSetFeedback(const sensor_msgs::msg::JoyFeedbackArray::SharedPtr msg);

private:
  void initForceFeedback();
  void uploadForceFeedback(const ff_effect &joy_effect);

  rclcpp::Logger logger_;
  std::string device_path_;
  int fd_;
};

#endif // ROS2_JOY_LINUX_FORCE_FEEDBACK_HPP_INCLUDED
