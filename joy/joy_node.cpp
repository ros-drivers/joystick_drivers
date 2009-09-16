#include <unistd.h>
#include <math.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "joy/Joy.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "txjoy");
  ros::NodeHandle n;ros::NodeHandle nh_param("~");

  ros::Publisher pub = n.advertise<joy::Joy>("joy", 1);
  std::string joy_dev;
  int deadzone;
  double repeatinterval = -1;  // in s.  use -1 for no repeat.
  nh_param.param<std::string>("dev", joy_dev, "/dev/input/js0");
  nh_param.param<int>("deadzone", deadzone, 2000);
  nh_param.param<double>("autorepeat_interval", repeatinterval, repeatinterval);  // in ms.

  joy::Joy joy_msg;
  js_event event;
  struct timeval tv;
  fd_set set;
  int joy_fd;

  // Big while loop opens, publishes
  while (n.ok())
  {
    bool first_fault = true;
		while (true)
		{
			if (!n.ok())
				goto cleanup;
		  joy_fd = open(joy_dev.c_str(), O_RDONLY);
      if (joy_fd != -1)
      {
				// There seems to be a bug in the driver or something where the
				// initial events that are to define the initial state of the
				// joystick are not the values of the joystick when it was opened
				// but rather the values of the joystick when it was last closed.
				// Opening then closing and opening again is a hack to get more
				// accurate initial state data.
				
				close(joy_fd);
        joy_fd = open(joy_dev.c_str(), O_RDONLY);
      }
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
      tv.tv_sec = trunc(repeatinterval);
      tv.tv_usec = (repeatinterval - tv.tv_sec) * 1e6; 
      FD_ZERO(&set);
      FD_SET(joy_fd, &set);

      int select_out = select(joy_fd+1, &set, NULL, NULL, repeatinterval >= 0 ? &tv : NULL);
      if (select_out < 0)
        break; // Joystick is probably closed. Not sure if this case is useful.

      if (FD_ISSET(joy_fd, &set))
      {
				if (read(joy_fd, &event, sizeof(js_event)) == -1 && errno != EAGAIN)
					break; // Joystick is probably closed. Definitely occurs.

				if (event.type & JS_EVENT_INIT)
					continue;
				switch(event.type)
				{
			  case JS_EVENT_BUTTON:
        case JS_EVENT_BUTTON | JS_EVENT_INIT:
					if(event.number >= joy_msg.get_buttons_size())
					{
						int old_size = joy_msg.get_buttons_size();
						joy_msg.set_buttons_size(event.number+1);
						for(unsigned int i=old_size;i<joy_msg.get_buttons_size();i++)
              joy_msg.buttons[i] = 0;
          }
          joy_msg.buttons[event.number] = (event.value ? 1 : 0);
					// if event is not an initial state event then publish it.
					// otherwise only use the event to build initial values into
					// message structure.
					if (!(event.type & JS_EVENT_INIT)) {
            pub.publish(joy_msg);
          }
          break;
        case JS_EVENT_AXIS:
        case JS_EVENT_AXIS | JS_EVENT_INIT:
          if(event.number >= joy_msg.get_axes_size())
          {
            int old_size = joy_msg.get_axes_size();
            joy_msg.set_axes_size(event.number+1);
            for(unsigned int i=old_size;i<joy_msg.get_axes_size();i++)
              joy_msg.axes[i] = 0.0;
          }
          joy_msg.axes[event.number] = (fabs(event.value) < deadzone) ? 0.0 :
            (-event.value / 32767.0);
					// if event is not an initial state event then publish it.
					// otherwise only use the event to build initial values into
					// message structure.
					if (!(event.type & JS_EVENT_INIT)) {
            pub.publish(joy_msg);
          }
          break;
        default:
          ROS_WARN("txjoy: Unknown event type. Please file a ticket. time=%u, value=%d, type=%Xh, number=%d", event.time, event.value, event.type, event.number);
          break;
				}
			}
			else if (repeatinterval >= 0) 
			{
				// Assume that all the JS_EVENT_INIT messages have arrived already.
				// This should be the case as the kernel sends them along as soon as
				// the device opens.
				pub.publish(joy_msg);
			}
		} // End of joystick open loop.

		close(joy_fd);
		if (n.ok())
			ROS_ERROR("Connection to joystick device lost unexpectedly. Will reopen.");
	}
cleanup:
	ROS_INFO("joy_node shut down.");

	return 0;
}

