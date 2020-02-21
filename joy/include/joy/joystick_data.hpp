#ifndef ROS2_JOY_JOYSTICK_DATA_HPP_INCLUDED
#define ROS2_JOY_JOYSTICK_DATA_HPP_INCLUDED

#include <string>

struct JoystickData
{
  std::string device_name;
  unsigned number_of_axes;
  unsigned number_of_buttons;

  virtual std::string toBasicInfoString() const;
  virtual std::string toDetailInfoString() const;
};

#endif // ROS2_JOY_JOYSTICK_DATA_HPP_INCLUDED
