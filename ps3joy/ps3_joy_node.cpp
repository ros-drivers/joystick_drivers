/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: Kevin Watts
// Modified from joy/joy.cpp by Jeremy Leibs

#include <unistd.h>
#include <math.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "joy/Joy.h"


using namespace std;

void *s_joy_func(void *);

class PS3Joy 
{
public:
  joy::Joy joy_msg;
  int joy_fd;
  int retry_count;
  string joy_dev;
  double deadzone;
  pthread_t joy_thread;

  ros::NodeHandle n_;
  ros::Publisher joy_pub_;


  void init()
  {
    n_.param("~dev", joy_dev, std::string("/dev/input/js0"));
    // Deadzone is given in units of output range from [-1, 1]
    n_.param("~deadzone", deadzone, 0.12);
    n_.param("~retry_count", retry_count, 100); // Retries this many times
    ROS_INFO("PS3 joystick device: %s.", joy_dev.c_str());
    ROS_INFO("PS3 joystick deadzone: %f.", deadzone);
    joy_pub_ = n_.advertise<joy::Joy>("joy",10);
  }

  void start()
  {
    int count = 0;
    joy_fd = open(joy_dev.c_str(), O_RDONLY);

    while (joy_fd <= 0 and n_.ok())
    {
      ROS_INFO("Attempting to open PS3 joystick: %s, attempt: %d", joy_dev.c_str(), ++count);
      if (count > retry_count)
      {
        ROS_FATAL("Couldn't open joystick %s after %d retries. Aborting.", 
                  joy_dev.c_str(), retry_count);
        ROS_BREAK();
      }
      sleep(1);

      joy_fd = open(joy_dev.c_str(), O_RDONLY);
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
    while (n_.ok())
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
            int old_buttons_size = joy_msg.buttons.size();

            joy_msg.buttons.resize(event.number + 1);
            for(unsigned int i = old_buttons_size; i<joy_msg.buttons.size(); i++)
              joy_msg.buttons[i] = 0;
          }
          if(event.value)
            joy_msg.buttons[event.number] = 1;
          else
            joy_msg.buttons[event.number] = 0;
          joy_pub_.publish(joy_msg);
          break;
        case JS_EVENT_AXIS:
          if(event.number >= joy_msg.get_axes_size())
          {
            int old_axis_size = joy_msg.axes.size();
            joy_msg.set_axes_size(event.number+1);
            for(unsigned int i=old_axis_size; i<joy_msg.get_axes_size(); i++)
              joy_msg.axes[i] = 0.0;
          }

	  // Normalize to +/-1 for PS3 axes 0-3
	  // PS3 axes 0-3 have range 0-256
	  // Axes >3 are inertial sensors, not currently supported in this node
	  double val = (- event.value) / 128.0;
          joy_msg.axes[event.number] = (fabs(val) < deadzone) ? 0.0 : val;
          joy_pub_.publish(joy_msg);
          break;
      }
    }
  }
};

void *s_joy_func(void *parent)
{
  ((PS3Joy *)parent)->joy_func();
  return NULL;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ps3_joy");

  PS3Joy ps3joy;
  ps3joy.init();

  ps3joy.start();
  ros::spin();
  ps3joy.stop();

  return 0;
}

