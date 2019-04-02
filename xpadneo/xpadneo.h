#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JoyFeedbackArray.h"
#include "sensor_msgs/JoyFeedback.h"

#include <string>

#include <linux/input.h>
#include "libevdev/libevdev.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

class Joystick{
    private:
        ros::NodeHandle _nodeHandle;
        ros::NodeHandle _privateNodeHandle;
        ros::Subscriber _feedbackSubscriber;
        ros::Publisher  _joyPublisher;

        std::string _joyDevName;

        int _joyFD;
        libevdev *_joyDEV;

        sensor_msgs::Joy _joyMessage;

    public:
        void feedbackCallback(const sensor_msgs::JoyFeedbackArrayConstPtr& msg);
        void cyclicCallback(const ros::TimerEvent &event);
        void init();

      
}