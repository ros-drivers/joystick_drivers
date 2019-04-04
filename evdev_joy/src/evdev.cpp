#include "evdev.h"

int main(int argc, char ** argv){
    ros::init(argc, argv, "evdev_joy");

    ModernJoystick joyer(ros::NodeHandle(""), ros::NodeHandle("~"));
    while(ros::ok())
    {
        joyer.run();
    }
}


ModernJoystick::ModernJoystick(ros::NodeHandle nh, ros::NodeHandle pnh):
_nodeHandle(nh),
_privateNodeHandle(pnh),
_joyDevName("/dev/input/event0"),
_buttonsMappingParam{"BTN_SOUTH",
                    "BTN_EAST",
                    "BTN_NORTH",
                    "BTN_WEST",
                    "BTN_TL",
                    "BTN_TR",
                    "BTN_SELECT",
                    "BTN_START",
                    "BTN_MODE",
                    "BTN_THUMBL",
                    "BTN_THUMBR"},
_axesMappingParam{  "ABS_X",
                    "ABS_Y",
                    "ABS_Z",
                    "ABS_RX",
                    "ABS_RY",
                    "ABS_RZ",
                    "ABS_HAT0X",
                    "ABS_HAT0Y"}
{
    this->init();
}

void ModernJoystick::init(){
    /**
     * Check Params
     */
    _privateNodeHandle.param<std::string>("device_file_path", _joyDevName, _joyDevName);
    _privateNodeHandle.param<std::vector<std::string>>("buttons_mapping", _buttonsMappingParam, _buttonsMappingParam);
    _privateNodeHandle.param<std::vector<std::string>>("axes_mapping", _axesMappingParam, _axesMappingParam);

    /**
     * Open Device
     */
    ModernJoystick::_joyFD = open(ModernJoystick::_joyDevName.c_str(), O_RDWR); //TODO: nonblocking read?
    ROS_ERROR_COND(_joyFD < 0, "Failed to open Device. %s", strerror(errno)); //TODO: Retry
    int err = libevdev_new_from_fd(_joyFD, &_joyDEV);
    ROS_ERROR_COND(err < 0, "Filed to init libevdev. %s", strerror(-err));
    //TODO: Exit
    
    /**
     * Quering Device Capabilitys 
     */
    ROS_INFO_STREAM("Input device name: " << libevdev_get_name(_joyDEV));
    if(libevdev_has_event_type(_joyDEV, EV_ABS)){
        ROS_INFO("Looks like a Controller, with Axes!");
    }

    //Checking and Calculating Mapping (Axes & Buttons)
    for(int i; i < _buttonsMappingParam.size(); i++){
        std::string buttonName = _buttonsMappingParam[i];
        int eventCode = libevdev_event_code_from_name(EV_KEY, buttonName.c_str());
        if(eventCode < 0){
            ROS_ERROR_STREAM("There is no Button Event (EV_KEY) '" << buttonName << "' in this Device. Skipping this Button!");
        }else{
            _buttonsMapping.insert(std::make_pair(eventCode, i));   
        }
    }
    

    for(int i; i < _axesMappingParam.size(); i++){
        std::string axesName = _axesMappingParam[i];
        int eventCode = libevdev_event_code_from_name(EV_ABS, axesName.c_str());
        if(eventCode < 0){
            ROS_ERROR_STREAM("There is no Axes Event (EV_ABS) '" << axesName << "' in this Device. Skipping this Axes!");
        }else{
            _axesMapping.insert(std::make_pair(eventCode, i)); 
             //Calculating Value Mapping for Axe
            int min = libevdev_get_abs_minimum(_joyDEV, eventCode);
            int max = libevdev_get_abs_maximum(_joyDEV, eventCode); 
            int absMax = fmax(abs(min),abs(max));
            _axesAbsMax.insert(std::make_pair(eventCode, absMax));
        }
    }


    
    /**
     * Initing ROS spefic things
     */
    _joyPublisher = _privateNodeHandle.advertise<sensor_msgs::Joy>("joy", 10);
    ROS_INFO_STREAM("Publishing to: " << _joyPublisher.getTopic());
    if(libevdev_has_event_type(_joyDEV, EV_FF)){
        _feedbackSubscriber = _privateNodeHandle.subscribe<sensor_msgs::JoyFeedbackArray>("set_feedback", 1, &ModernJoystick::feedbackCallback, this);
        ROS_INFO_STREAM("Joystick seems to have Force-Feedback, Subscribing to: " << _feedbackSubscriber.getTopic());
    }

    /**
     * Inting The Arrays
     */
    _joyMessage.axes.resize(_axesMappingParam.size());
    _joyMessage.buttons.resize(_buttonsMappingParam.size());
    
   
}

void ModernJoystick::run(){
    struct input_event ev;
    readJoy(ev);
    updateMessage(ev);  
    //Send
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "joy_link"; //TODO: parametrice
    _joyMessage.header = header;
    _joyPublisher.publish(_joyMessage);
}

void ModernJoystick::feedbackCallback(const sensor_msgs::JoyFeedbackArrayConstPtr& msg){
    //For Each Feedback:
    for(sensor_msgs::JoyFeedback feedback : msg->array){
        struct ff_effect effect;
        
    }


    
}

ModernJoystick::~ModernJoystick(){
    libevdev_free(_joyDEV);
    close(_joyFD); //TODO: ERROR
}





void ModernJoystick::readJoy(struct input_event & ev){
    int rs = libevdev_next_event(_joyDEV, libevdev_read_flag::LIBEVDEV_READ_FLAG_NORMAL | libevdev_read_flag::LIBEVDEV_READ_FLAG_BLOCKING, &ev);
    if(rs == libevdev_read_status::LIBEVDEV_READ_STATUS_SUCCESS){
       return;
    }else if(rs == libevdev_read_status::LIBEVDEV_READ_STATUS_SYNC){
        //Resync in Sync-Mode
        ROS_WARN("Controller out of sync!");
        reSyncJoy();

    }else{
        if(rs == -EAGAIN){
            //No more events
        }else{
            //Error
        }
    }

}


void ModernJoystick::updateMessage(const struct input_event & ev){
    if(ev.type == EV_KEY && _buttonsMapping.count(ev.code) == 1){
        _joyMessage.buttons[_buttonsMapping[ev.code]] = ev.value;
    }else if(ev.type == EV_ABS && _axesMapping.count(ev.code) == 1){
        _joyMessage.axes[_axesMapping[ev.code]] = mapAxesValue(ev.value, ev.code);
    }//Everything else is not handled by this Node
        //Else means: (1) Another Key, or (2) a not mapped Code
}


void ModernJoystick::reSyncJoy(){
    struct input_event ev;
    int rc = LIBEVDEV_READ_STATUS_SYNC;
    ROS_WARN("resyncing Controller...");
    while (rc == LIBEVDEV_READ_STATUS_SYNC) {
        rc = libevdev_next_event(_joyDEV, LIBEVDEV_READ_FLAG_SYNC, &ev);
        if (rc < 0) {
            if (rc != -EAGAIN){
                ROS_ERROR("Error while reading Event from Controller: %s", strerror(-rc));
            }
            return;
        }

        //TODO:Sending Messages!
    }
    updateMessage(ev);
    ROS_WARN("Controller is resynct, maybe you've lost some Messages.");
}

float ModernJoystick::mapAxesValue(int value, int evCode){
    int absMax = _axesAbsMax[evCode];
    float percVal = float(value) / float(absMax);
    return percVal;
}