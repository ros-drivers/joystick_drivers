#include <joy/joystick_data.hpp>

std::string JoystickData::toBasicInfoString() const
{
  return device_name;
}

std::string JoystickData::toDetailInfoString() const
{
  return toBasicInfoString() + ", "
          + std::to_string(number_of_axes) + " axes, "
          + std::to_string(number_of_buttons) + " buttons";
}
