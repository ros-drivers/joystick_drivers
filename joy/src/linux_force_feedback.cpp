#include <joy/linux_force_feedback.hpp>

#include <fcntl.h>
#include <linux/joystick.h>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <sys/ioctl.h>
#include <unistd.h>

LinuxForceFeedbackDevice::LinuxForceFeedbackDevice(const std::string &device_path, rclcpp::Logger parent_logger)
: logger_(parent_logger.get_child("feedback")),
  device_path_(device_path),
  fd_(-1)
{

}

LinuxForceFeedbackDevice::~LinuxForceFeedbackDevice()
{
  close();
}

void LinuxForceFeedbackDevice::open()
{
  if (isOpen()) {
    RCLCPP_ERROR_STREAM(logger_, "Device already open: " << device_path_);
    return;
  }

  fd_ = ::open(device_path_.c_str(), O_RDWR);
  if (fd_ == -1) {
      RCLCPP_ERROR_STREAM(logger_, "Cannot open device " << device_path_
                          << ". Error " << errno << ": " << strerror(errno));
  } else {
    initForceFeedback();
  }
}

void LinuxForceFeedbackDevice::close()
{
  if (isOpen()) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool LinuxForceFeedbackDevice::isOpen() const
{
  return fd_ != -1;
}

void LinuxForceFeedbackDevice::initForceFeedback()
{
  /* Set the gain of the device*/
  int gain = 100;           /* between 0 and 100 */
  struct input_event ie;      /* structure used to communicate with the driver */
  ie.type = EV_FF;
  ie.code = FF_GAIN;
  ie.value = 0xFFFFUL * gain / 100;

  if (::write(fd_, &ie, sizeof(ie)) == -1) {
    RCLCPP_ERROR_STREAM(logger_, "Could not write joystick force feedback"
                        << ". Error " << errno << ": " << strerror(errno));
    return;
  }

  struct ff_effect joy_effect;
  joy_effect.id = -1;//0;
  joy_effect.direction = 0;//down
  joy_effect.type = FF_RUMBLE;
  joy_effect.u.rumble.strong_magnitude = 0;
  joy_effect.u.rumble.weak_magnitude = 0;
  joy_effect.replay.length = 1000;
  joy_effect.replay.delay = 0;

  uploadForceFeedback(joy_effect);
}

void LinuxForceFeedbackDevice::uploadForceFeedback(const ff_effect &joy_effect)
{
  struct input_event ie;
  ie.type = EV_FF;
  ie.code = joy_effect.id;
  ie.value = 3; // TODO: magic number?

  if (::write(fd_, &ie, sizeof(ie)) == -1) {
    RCLCPP_ERROR_STREAM(logger_, "Could not write joystick force feedback"
                        << ". Error " << errno << ": " << strerror(errno));
    return;
  }

  // upload the effect
  if (::ioctl(fd_, EVIOCSFF, &joy_effect) == -1) {
    RCLCPP_ERROR_STREAM(logger_, "Could not upload joystick force feedback"
                        << ". Error " << errno << ": " << strerror(errno));
  }
}

void LinuxForceFeedbackDevice::handleSetFeedback(const sensor_msgs::msg::JoyFeedbackArray::SharedPtr msg)
{
  if (!isOpen()) return;

  for (const sensor_msgs::msg::JoyFeedback &effect : msg->array) {
    switch (effect.type) {
      case sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE:
        struct ff_effect joy_effect;

        joy_effect.direction = 0; // 0 = down
        joy_effect.type = FF_RUMBLE;
        if (effect.id == 0)
          joy_effect.u.rumble.strong_magnitude = ((float)(1<<15)) * effect.intensity;
        else
          joy_effect.u.rumble.weak_magnitude = ((float)(1<<15)) * effect.intensity;

        joy_effect.replay.length = 1000;
        joy_effect.replay.delay = 0;

        uploadForceFeedback(joy_effect);
        break;
      default:
        RCLCPP_WARN_STREAM_ONCE(logger_, "Force feedback effect with type=" << effect.type << " not implemented, yet."
                                << " Won't report any further missed effects.");
    }
  }
}
