/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Stuart Glaser
 */

#include <stdio.h>
#include "ros/node.h"
#include "spnav.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "joy/Joy.h"

#define FULL_SCALE (512.0)
//Used to scale joystick output to be in [-1, 1].  Estimated from data, and not necessarily correct.

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  ros::Node node("spacenav");
  node.advertise<geometry_msgs::Vector3>("/spacenav/offset", 2);
  node.advertise<geometry_msgs::Vector3>("/spacenav/rot_offset", 2);
  node.advertise<geometry_msgs::Twist>("/spacenav/twist", 2);
  node.advertise<joy::Joy>("/spacenav/joy", 2);

  if (spnav_open() == -1)
  {
    ROS_ERROR("Could not open the space navigator device.  Did you remember to run spacenavd (as root)?");

    return 1;
  }

  spnav_event sev;
  int ret;
  int no_motion_count = 0;
  joy::Joy joystick_msg;
  joystick_msg.axes.resize(6);
  joystick_msg.buttons.resize(2);
  while (node.ok())
  {
    ret = spnav_poll_event(&sev);
    spnav_remove_events(SPNAV_EVENT_MOTION);

    if (ret == 0)
    {
      if (++no_motion_count > 30)
      {
        no_motion_count = 0;
        geometry_msgs::Vector3 offset_msg;
        offset_msg.x = offset_msg.y = offset_msg.z = 0;
        node.publish("/spacenav/offset", offset_msg);

        geometry_msgs::Vector3 rot_offset_msg;
        rot_offset_msg.x = rot_offset_msg.y = rot_offset_msg.z = 0;
        node.publish("/spacenav/rot_offset", rot_offset_msg);

        geometry_msgs::Twist twist_msg;
        twist_msg.linear = offset_msg;
        twist_msg.angular = rot_offset_msg;
        node.publish("spacenav/twist", twist_msg);

        joystick_msg.axes[0] = offset_msg.x / FULL_SCALE;
        joystick_msg.axes[1] = offset_msg.y / FULL_SCALE;
        joystick_msg.axes[2] = offset_msg.z / FULL_SCALE;
        joystick_msg.axes[3] = rot_offset_msg.x / FULL_SCALE;
        joystick_msg.axes[4] = rot_offset_msg.y / FULL_SCALE;
        joystick_msg.axes[5] = rot_offset_msg.z / FULL_SCALE;
      }
    }
    if (sev.type == SPNAV_EVENT_MOTION)
    {
      geometry_msgs::Vector3 offset_msg;
      offset_msg.x = sev.motion.z;
      offset_msg.y = -sev.motion.x;
      offset_msg.z = sev.motion.y;
      node.publish("/spacenav/offset", offset_msg);

      geometry_msgs::Vector3 rot_offset_msg;
      rot_offset_msg.x = sev.motion.rz;
      rot_offset_msg.y = -sev.motion.rx;
      rot_offset_msg.z = sev.motion.ry;

      //printf("%lf  %lf  %lf\n", rot_offset_msg.x, rot_offset_msg.y, rot_offset_msg.z);
      node.publish("/spacenav/rot_offset", rot_offset_msg);

      geometry_msgs::Twist twist_msg;
      twist_msg.linear = offset_msg;
      twist_msg.angular = rot_offset_msg;
      node.publish("spacenav/twist", twist_msg);

      joystick_msg.axes[0] = offset_msg.x / FULL_SCALE;
      joystick_msg.axes[1] = offset_msg.y / FULL_SCALE;
      joystick_msg.axes[2] = offset_msg.z / FULL_SCALE;
      joystick_msg.axes[3] = rot_offset_msg.x / FULL_SCALE;
      joystick_msg.axes[4] = rot_offset_msg.y / FULL_SCALE;
      joystick_msg.axes[5] = rot_offset_msg.z / FULL_SCALE;

      no_motion_count = 0;
    }
    else if (sev.type == SPNAV_EVENT_BUTTON)
    {
      //printf("type, press, bnum = <%d, %d, %d>\n", sev.button.type, sev.button.press, sev.button.bnum);
      joystick_msg.buttons[sev.button.bnum] = sev.button.press;
    }
    node.publish("/spacenav/joy", joystick_msg);
    usleep(1000);
  }

  node.unadvertise("/spacenav/offset");



  return 0;
}
