#ifndef ROS2_JOY_LINUX_HELPER_HPP_INCLUDED
#define ROS2_JOY_LINUX_HELPER_HPP_INCLUDED

#include "joystick_data.hpp"
#include <rclcpp/logger.hpp>
#include <string>
#include <vector>

struct LinuxJoystickData : public JoystickData
{
  std::string device_path;

  std::string toBasicInfoString() const;
};

bool fillJoystickData(const std::string &device_path, LinuxJoystickData &data, rclcpp::Logger logger);

std::vector<LinuxJoystickData> getJoysticks(rclcpp::Logger logger);

#endif // ROS2_JOY_LINUX_HELPER_HPP_INCLUDED
