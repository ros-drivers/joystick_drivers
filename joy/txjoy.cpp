#include <unistd.h>
#include <math.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "joy/Joy.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "txjoy");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<joy::Joy>("joy", 1);
  std::string joy_dev;
  int deadzone = 0;
  n.param<std::string>("~dev", joy_dev, "/dev/input/js0");
  n.param<int>("~deadzone", deadzone, 2000);
  int joy_fd = open(joy_dev.c_str(), O_RDONLY);
  if (joy_fd <= 0)
  {
    ROS_FATAL("couldn't open joystick %s.", joy_dev.c_str());
    ROS_BREAK();
  }
  joy::Joy joy_msg;
  js_event event;
  struct timeval tv;
  fd_set set;
  while (n.ok())
  {
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // wait 100ms for something to happen
    FD_ZERO(&set);
    FD_SET(joy_fd, &set);
    if (select(joy_fd+1, &set, NULL, NULL, &tv) == 1 && FD_ISSET(joy_fd, &set))
    {
      read(joy_fd, &event, sizeof(js_event));
      if (event.type & JS_EVENT_INIT)
        continue;
      switch(event.type)
      {
        case JS_EVENT_BUTTON:
          if(event.number >= joy_msg.get_buttons_size())
          {
            joy_msg.set_buttons_size(event.number+1);
            for(unsigned int i=0;i<joy_msg.get_buttons_size();i++)
              joy_msg.buttons[i] = 0;
          }
          joy_msg.buttons[event.number] = (event.value ? 1 : 0);
          pub.publish(joy_msg);
          break;
        case JS_EVENT_AXIS:
          if(event.number >= joy_msg.get_axes_size())
          {
            joy_msg.set_axes_size(event.number+1);
            for(unsigned int i=0;i<joy_msg.get_axes_size();i++)
              joy_msg.axes[i] = 0.0;
          }
          joy_msg.axes[event.number] = (fabs(event.value) < deadzone) ? 0.0 :
            (-event.value / 32767.0);
          pub.publish(joy_msg);
          break;
      }
    }
  }
  close(joy_fd);
  return 0;
}

