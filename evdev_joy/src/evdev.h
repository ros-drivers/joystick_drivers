#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JoyFeedbackArray.h"
#include "sensor_msgs/JoyFeedback.h"



#include <string>

#include <linux/input.h>
#include <libevdev-1.0/libevdev/libevdev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <list>

#include <cmath>

#include <sys/select.h>

class ModernJoystick{
    private:
        //ROS
        ros::NodeHandle _nodeHandle;
        ros::NodeHandle _privateNodeHandle;
        ros::Subscriber _feedbackSubscriber;
        ros::Publisher  _joyPublisher;
        ros::Timer _sendTimer;
        //Params
        std::string _joyDevName;
        std::vector<std::string> _buttonsMappingParam;
        std::vector<std::string> _axesMappingParam;
        int _maxSendFrequency;


        //Device
        int _joyFD;
        libevdev *_joyDEV;
        fd_set _fdSET;

        sensor_msgs::Joy _joyMessage;
        std::map<int,int> _buttonsMapping;
        std::map<int,int> _axesMapping;
        std::map<int, int> _axesAbsMax; //maximum abs value, of min/max -> to calculate the perc.
        std::map<int, short> _feedbackDeviceID; //maps ArraySlot to Device-Feedback-Slot

        //libevdev
        bool readJoy(struct input_event & ev);
        void reSyncJoy();
        void updateMessage(const struct input_event & ev);
        float mapAxesValue(int value, int evCode);

        //feedback
        void addEffect(struct ff_effect & effect);
        void playEffect(short effectID);
        void stopEffect(short effectID);
        void removeEffect(short effectID);

        void publishJoyMessage();

        void connect();
    public:
        ModernJoystick(ros::NodeHandle nh, ros::NodeHandle pnh);
        ~ModernJoystick();
        void feedbackCallback(const sensor_msgs::JoyFeedbackArrayConstPtr& msg);
        void timerCallback(const ros::TimerEvent & event);
        void run();
        void init();

        enum FeedBackID{
            RUMBLE_HEAVY, RUMBLE_LIGHT 
        };

      
};


