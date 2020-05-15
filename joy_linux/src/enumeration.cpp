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

#include <joy_linux/enumeration.hpp>

#include <dirent.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <rclcpp/logging.hpp>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <functional>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

std::ostream & operator<<(std::ostream & os, const JoystickData & joystick)
{
  os << "device=" << joystick.device_path <<
    " name=" << joystick.device_name <<
    " axes=" << joystick.number_of_axes <<
    " buttons=" << joystick.number_of_buttons;
  return os;
}

class Defer
{
public:
  explicit Defer(std::function<void(void)> f)
  : f_(f) {}
  ~Defer() {f_();}

private:
  std::function<void(void)> f_;
};

bool isCharacterDeviceFile(const std::string & path)
{
  struct stat statbuf;
  if (::stat(path.c_str(), &statbuf) == 0) {
    return S_ISCHR(statbuf.st_mode);  // input devices are character devices
  }
  return false;
}

bool fillJoystickData(const std::string & device_path, JoystickData & data, rclcpp::Logger logger)
{
  if (!isCharacterDeviceFile(device_path)) {return false;}

  int fd = ::open(device_path.c_str(), O_RDONLY);
  if (fd == -1) {
    RCLCPP_ERROR_STREAM(
      logger, "Cannot open " << device_path <<
        ". Error " << errno << ": " << strerror(errno));
    return false;
  }
  Defer deferred_close([&] {::close(fd);});

  data.device_path = device_path;

  // get number of axes
  int ioctl_result;
  char number_of_axes;
  ioctl_result = ::ioctl(fd, JSIOCGAXES, &number_of_axes);
  if (ioctl_result == -1) {
    RCLCPP_ERROR_STREAM(
      logger, "Cannot get number of axes from " << device_path <<
        ". Error " << errno << ": " << strerror(errno));
    return false;
  }
  data.number_of_axes = number_of_axes;

  // get number of buttons
  char number_of_buttons;
  ioctl_result = ::ioctl(fd, JSIOCGBUTTONS, &number_of_buttons);
  if (ioctl_result == -1) {
    RCLCPP_ERROR_STREAM(
      logger, "Cannot get number of buttons from " << device_path <<
        ". Error " << errno << ": " << strerror(errno));
    return false;
  }
  data.number_of_buttons = number_of_buttons;

  // get joystick name
  char joystick_name[128];
  ioctl_result = ::ioctl(fd, JSIOCGNAME(sizeof(joystick_name)), joystick_name);
  // ioctl_result seems to be the length of joystick_name including the trailing zero.
  // Could it be used to detect if the provided buffer was too small to hold the full name?
  // (The return value for JSIOCGNAME is not documented, maybe this is driver specific.)
  if (ioctl_result == -1) {
    RCLCPP_ERROR_STREAM(
      logger, "Cannot get device name from " << device_path <<
        ". Error " << errno << ": " << strerror(errno));
    return false;
  }
  data.device_name = joystick_name;

  return true;
}

std::vector<JoystickData> getJoysticks(rclcpp::Logger logger)
{
  std::string path = "/dev/input/";
  std::vector<JoystickData> joysticks;
  struct dirent * entry;

  DIR * dev_dir = ::opendir(path.c_str());
  if (dev_dir == NULL) {
    RCLCPP_ERROR_STREAM(
      logger, "Cannot open " << path <<
        ". Error " << errno << ": " << strerror(errno));
    return joysticks;
  }
  Defer deffered_close_dir([&] {::closedir(dev_dir);});

  while ((entry = readdir(dev_dir)) != NULL) {
    // filter entries
    if (std::strncmp(entry->d_name, "js", 2) != 0) {  // skip device if it's not a joystick
      continue;
    }

    std::string current_path = path + entry->d_name;
    JoystickData joystick_data;
    if (fillJoystickData(current_path, joystick_data, logger)) {
      RCLCPP_INFO_STREAM(logger, "Found joystick: " << joystick_data);
      joysticks.push_back(joystick_data);
    }
  }

  return joysticks;
}
