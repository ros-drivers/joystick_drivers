#include <joy/linux_helper.hpp>

#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <functional>
#include <linux/joystick.h>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

class Defer
{
public:
  Defer(std::function<void (void)> f) : f_(f) {}
  ~Defer() { f_(); }
private:
  std::function<void (void)> f_;
};

bool isCharacterDeviceFile(const std::string &path)
{
  struct stat statbuf;
  if (::stat(path.c_str(), &statbuf) == 0) {
    return S_ISCHR(statbuf.st_mode); // input devices are character devices
  }
  return false;
}

std::string LinuxJoystickData::toBasicInfoString() const
{
  return device_name + " (" + device_path + ")";
}

bool fillJoystickData(const std::string &device_path, LinuxJoystickData &data, rclcpp::Logger logger)
{
  if (!isCharacterDeviceFile(device_path)) return false;

  int fd = ::open(device_path.c_str(), O_RDONLY);
  if (fd == -1) {
    RCLCPP_ERROR_STREAM(logger, "Cannot open " << device_path
                        << ". Error " << errno << ": " << strerror(errno));
    return false;
  }
  Defer deferred_close([&]{ ::close(fd); });

  data.device_path = device_path;

  // get number of axes
  int ioctl_result;
  char number_of_axes;
  ioctl_result = ::ioctl(fd, JSIOCGAXES, &number_of_axes);
  if (ioctl_result == -1) {
    RCLCPP_ERROR_STREAM(logger, "Cannot get number of axes from " << device_path
                        << ". Error " << errno << ": " << strerror(errno));
    return false;
  }
  data.number_of_axes = number_of_axes;

  // get number of buttons
  char number_of_buttons;
  ioctl_result = ::ioctl(fd, JSIOCGBUTTONS, &number_of_buttons);
  if (ioctl_result == -1) {
    RCLCPP_ERROR_STREAM(logger, "Cannot get number of buttons from " << device_path
                        << ". Error " << errno << ": " << strerror(errno));
    return false;
  }
  data.number_of_buttons = number_of_buttons;

  // get joystick name
  char joystick_name[128];
  ioctl_result = ::ioctl(fd, JSIOCGNAME(sizeof(joystick_name)), joystick_name);
  // TODO: ioctl_result seems to be the length of joystick_name including the trailing zero.
  // Could it be used to detect if the provided buffer was too small to hold the full name?
  // (The return value for JSIOCGNAME is not documented, maybe this is driver specific.)
  if (ioctl_result == -1) {
      RCLCPP_ERROR_STREAM(logger, "Cannot get device name from " << device_path
                          << ". Error " << errno << ": " << strerror(errno));
      return false;
  }
  data.device_name = joystick_name;

  return true;
}

std::vector<LinuxJoystickData> getJoysticks(rclcpp::Logger logger)
{
  std::string path = "/dev/input/";
  std::vector<LinuxJoystickData> joysticks;
  struct dirent *entry;

  DIR *dev_dir = ::opendir(path.c_str());
  if (dev_dir == NULL) {
    RCLCPP_ERROR_STREAM(logger, "Cannot open " << path
                        << ". Error " << errno << ": " << strerror(errno));
    return joysticks;
  }
  Defer deffered_close_dir([&]{ ::closedir(dev_dir); });

  while ((entry = readdir(dev_dir)) != NULL) {
    // filter entries
    if (std::strncmp(entry->d_name, "js", 2) != 0) {  // skip device if it's not a joystick
      continue;
    }

    std::string current_path = path + entry->d_name;
    LinuxJoystickData joystick_data;
    if (fillJoystickData(current_path, joystick_data, logger)) {
      RCLCPP_INFO_STREAM(logger, "Found joystick: " << joystick_data.toDetailInfoString());
      joysticks.push_back(joystick_data);
    }
  }

  return joysticks;
}
