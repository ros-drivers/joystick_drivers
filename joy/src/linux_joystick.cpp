#include <joy/linux_joystick.hpp>
#include <joy/linux_force_feedback.hpp>

#include <algorithm>
#include <cstring>
#include <fcntl.h>
#include <linux/joystick.h>
#include <stdexcept>
#include <unistd.h>

// forward declarations
bool isCharacterDeviceFile(const std::string &filepath);
bool getJoystickName(const std::string &device_file, std::string &joystick_name);

LinuxJoystick::LinuxJoystick(rclcpp::NodeOptions options)
: JoyNode(options),
  fd_(-1)
{
  static constexpr char DEFAULT_DEVICE[] = "/dev/input/js0";

  device_path_ = this->declare_parameter("device", "");

  if (device_path_.empty()) {
    // if no device path was specified, check for device_name
    std::string device_name = this->declare_parameter("device_name", "");
    if (device_name.empty()) {
      device_path_ = DEFAULT_DEVICE;
      RCLCPP_WARN_STREAM(this->get_logger(),
        "Neither device nor device_name were specified, will use default device: " << device_path_);
    } else {
      // enumerate all joysticks and check if one matches device_name
      RCLCPP_INFO_STREAM(this->get_logger(), "Looking for joystick with name: " << device_name);
      std::vector<LinuxJoystickData> joysticks = getJoysticks(this->get_logger());
      auto joystick_it = std::find_if(joysticks.begin(), joysticks.end(),
        [&device_name](const LinuxJoystickData &data){ return data.device_name == device_name; });
      if (joystick_it != joysticks.end()) { // found a joystick matching device_name
        device_path_ = joystick_it->device_path;
        RCLCPP_INFO_STREAM(this->get_logger(), "Will use device: " << device_path_);
      } else {
        device_path_ = DEFAULT_DEVICE;
        RCLCPP_WARN_STREAM(this->get_logger(),
          "Joystick with name " << device_name << " not found, will use default device: " << device_path_);
      }
    }
  }

  std::string ff_device = this->declare_parameter("ff_device", "");
  if (!ff_device.empty()) {
    force_feedback_device_ = std::make_unique<LinuxForceFeedbackDevice>(ff_device, this->get_logger());
  }
}

LinuxJoystick::~LinuxJoystick()
{
  close();
}

bool LinuxJoystick::tryOpen(JoystickData &joystick_data)
{
  // There seems to be a bug in the driver or something where the
  // initial events that are to define the initial state of the
  // joystick are not the values of the joystick when it was opened
  // but rather the values of the joystick when it was last closed.
  // Opening then closing and opening again is a hack to get more
  // accurate initial state data.

  // fillJoystickData will open and close the device
  LinuxJoystickData linux_joystick_data;
  if (!fillJoystickData(device_path_, linux_joystick_data, this->get_logger())) {
    return false;
  }
  joystick_data = linux_joystick_data;

  for (size_t i = 0; i < linux_joystick_data.number_of_axes; ++i)
    pending_axis_init_events_.emplace_back(i);
  for (size_t i = 0; i < linux_joystick_data.number_of_buttons; ++i)
    pending_button_init_events_.emplace_back(i);

  fd_ = ::open(device_path_.c_str(), O_RDONLY);
  if (fd_ == -1) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot open " << device_path_
                        << ". Error " << errno << ": " << strerror(errno));
    return false;
  }

  if (force_feedback_device_) force_feedback_device_->open();

  return true;
}

void LinuxJoystick::close()
{
  if (isOpen()) {
    ::close(fd_);
    RCLCPP_INFO(this->get_logger(), "Joystick closed");
    fd_ = -1;
  }
}

bool LinuxJoystick::isOpen() const
{
  return fd_ != -1;
}

void LinuxJoystick::handleSetFeedback(const sensor_msgs::msg::JoyFeedbackArray::SharedPtr msg)
{
  if (force_feedback_device_) force_feedback_device_->handleSetFeedback(msg);
}

ProcessEventsResult LinuxJoystick::processEvents()
{
  struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  fd_set set;
  FD_ZERO(&set);
  FD_SET(fd_, &set);

  int select_result = ::select(fd_ + 1, &set, nullptr, nullptr, &timeout);
  if (select_result > 0 && FD_ISSET(fd_, &set)) { // the file descriptor has data to read
    js_event event;
    if (::read(fd_, &event, sizeof(event)) == -1) {
      if (errno == EAGAIN) { // read would block
        return ProcessEventsResult::Continue; // try again
      } else if (errno == EINTR) { // read was interrupted by a signal
        return ProcessEventsResult::Abort;
      }
      RCLCPP_ERROR_STREAM(this->get_logger(), "Error " << errno << " while reading from the joystick device: " << strerror(errno));
      return ProcessEventsResult::Abort;
    }

    return processJoystickEvent(event);
  } else if (select_result == 0) { // a timeout occured
    return ProcessEventsResult::Continue;
  } else if (select_result < 0) { // an error occured
    if (errno == EINTR) { // select was interrupted by a signal
      return ProcessEventsResult::Abort;
    }
    RCLCPP_ERROR_STREAM(this->get_logger(), "Error " << errno << " while waiting for the joystick device: " << strerror(errno));
    return ProcessEventsResult::Abort;
  }

  // in all other cases: continue
  return ProcessEventsResult::Continue;
}

ProcessEventsResult LinuxJoystick::processJoystickEvent(const js_event &event)
{
  switch (event.type) {
    case JS_EVENT_AXIS:
      updateAxis(event.number, event.value);
      return ProcessEventsResult::PublishSoon;
    case JS_EVENT_BUTTON:
      updateButton(event.number, (event.value ? 1:0));
      return ProcessEventsResult::PublishNow;
    case JS_EVENT_BUTTON | JS_EVENT_INIT:
      updateButton(event.number, (event.value ? 1:0));
      pending_button_init_events_.remove(event.number);
      //RCLCPP_INFO_STREAM(this->get_logger(), "Button Init " << +event.number);
      return checkInitEvents();
    case JS_EVENT_AXIS | JS_EVENT_INIT:
      updateAxis(event.number, event.value);
      pending_axis_init_events_.remove(event.number);
      //RCLCPP_INFO_STREAM(this->get_logger(), "Axis Init " << +event.number);
      return checkInitEvents();
  }

  return ProcessEventsResult::Continue;
}

ProcessEventsResult LinuxJoystick::checkInitEvents()
{
  if (pending_axis_init_events_.empty() && pending_button_init_events_.empty()) {
    //RCLCPP_INFO(this->get_logger(), "All init events received");
    startAutorepeatPublishing();
    return ProcessEventsResult::PublishNow;
  }
  return ProcessEventsResult::Continue;
}
