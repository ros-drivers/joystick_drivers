#include <unistd.h>
#include <math.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include "ros/node.h"
#include "joy/Joy.h"

using namespace std;

void *s_joy_func(void *);
using namespace ros;

class Joy : public Node
{
public:
  joy::Joy joy_msg;
  int joy_fd;
  string joy_dev;
  int deadzone;
  pthread_t joy_thread;

  Joy() : Node("joy")
  {
    param<string>("~dev", joy_dev, "/dev/input/js0");
    param<int>("~deadzone", deadzone, 2000);
    printf("dev:%s\n", joy_dev.c_str());
    printf("deadzone:%d\n", deadzone);
    advertise<joy::Joy>("joy",10);
  }
  void start()
  {
    joy_fd = open(joy_dev.c_str(), O_RDONLY);
    if (joy_fd <= 0)
    {
      ROS_FATAL("couldn't open joystick %s.", joy_dev.c_str());
      ROS_BREAK();
    }
    pthread_create(&joy_thread, NULL, s_joy_func, this);
  }
  void stop()
  {
    pthread_cancel(joy_thread);
    pthread_join(joy_thread,NULL);
    close(joy_fd);
  }
  void joy_func()
  {
    js_event event;
    while (ok())
    {
      pthread_testcancel();
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
          if(event.value)
            joy_msg.buttons[event.number] = 1;
          else
            joy_msg.buttons[event.number] = 0;
          publish("joy", joy_msg);
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
          cout << "axis: " << (int) event.number << ", value: " << joy_msg.axes[event.number] << endl;
          publish("joy", joy_msg);
          break;
      }
    }
  }
};

void *s_joy_func(void *parent)
{
  ((Joy *)parent)->joy_func();
  return NULL;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  Joy joy;
  joy.start();
  joy.spin();
  joy.stop();
  return 0;
}

