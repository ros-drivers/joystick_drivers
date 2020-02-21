#ifndef ROS2_JOY_LINUX_JOYSTICK_HPP_INCLUDED
#define ROS2_JOY_LINUX_JOYSTICK_HPP_INCLUDED

#include "joy_node.hpp"
#include "linux_helper.hpp"
#include <list>
#include <memory>

struct js_event;
class LinuxForceFeedbackDevice;

class LinuxJoystick : public JoyNode
{
public:
    LinuxJoystick(rclcpp::NodeOptions options);
    ~LinuxJoystick();

private:
    bool tryOpen(JoystickData &joystick_data);
    void close();
    bool isOpen() const;

    ProcessEventsResult processEvents();
    ProcessEventsResult processJoystickEvent(const js_event &event);
    ProcessEventsResult checkInitEvents();

    // force feedback
    void handleSetFeedback(const sensor_msgs::msg::JoyFeedbackArray::SharedPtr msg);
    std::unique_ptr<LinuxForceFeedbackDevice> force_feedback_device_;

    // device information
    std::string device_path_;
    int fd_;

    // initial events
    std::list<unsigned> pending_axis_init_events_;
    std::list<unsigned> pending_button_init_events_;
};

#endif // ROS2_JOY_LINUX_JOYSTICK_HPP_INCLUDED
