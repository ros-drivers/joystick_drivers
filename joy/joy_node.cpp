#include <unistd.h>
#include <math.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "joy/Joy.h"
#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "txjoy");
  ros::NodeHandle n;
	ros::NodeHandle nh_param("~");

  // Big while loop opens, publishes
  ros::Publisher pub = n.advertise<joy::Joy>("joy", 1);

  joy::Joy joy_msg;
  js_event event;
  struct timeval tv;
  fd_set set;
  int joy_fd;

  while (n.ok())
  {
    std::string joy_dev;
    int deadzone;
    nh_param.param<std::string>("dev", joy_dev, "/dev/input/js0");
    nh_param.param<int>("deadzone", deadzone, 2000);
                                                                   
    bool first_fault = true;
		while (true)
		{
			if (!n.ok())
				goto cleanup;
		  joy_fd = open(joy_dev.c_str(), O_RDONLY);
      if (joy_fd != -1)
				break;
			if (first_fault)
			{
				ROS_ERROR("Couldn't open joystick %s. Will retry every second.", joy_dev.c_str());
				first_fault = false;
			}
      sleep(1.0);
		}

    ROS_INFO("Opened joystick: %s. Deadzone: %i.", joy_dev.c_str(), deadzone);

    while (n.ok()) 
    {
      tv.tv_sec = 0;
      tv.tv_usec = 50000; // wait 50ms for something to happen
      FD_ZERO(&set);
      FD_SET(joy_fd, &set);

      int select_val = select(joy_fd+1, &set, NULL, NULL, &tv);
      if (select_val < 0)
        break; // Joystick is probably closed. Not sure if this case is useful.

      if (select_val == 1 && FD_ISSET(joy_fd, &set))
      {
        if (read(joy_fd, &event, sizeof(js_event)) == -1 && errno != EAGAIN)
					break; // Joystick is probably closed. Definitely occurs.

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
		if (n.ok())
			ROS_ERROR("Connection to joystick device lost unexpectedly. Will reopen.");
  }
cleanup:
	ROS_INFO("Joystick shutting down.");

  return 0;
}

