#include "xpadneo.h"

Joystick::Joystick():
{
    this->init();
}

void Joystick::init(){
    
    Joystick::_joyFD = open(Joystick::_joyDevName.c_str, O_RDWR);
    int res = libevdev_new_from_fd(_joyFD, &_joyDEV);
    ROS_ERROR_COND(res < 0, "Filed to init libevdev.");
    //TODO: Exit
    ROS_INFO_STREAM("Input device name: " << libevdev_get_name(_joyDEV));

}

void Joystick::cyclicCallback(const ros::TimerEvent &event){
    struct input_event ev;
    int rc = libevdev_next_event(_joyDEV, LIBEVDEV_READ_FLAG_NORMAL, &ev);
    

}