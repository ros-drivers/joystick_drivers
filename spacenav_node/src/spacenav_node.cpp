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
#include <vector>
#include "ros/node_handle.h"
#include "ros/param.h"
#include "spnav.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

/** Ensure that the vector parameter has three components.
 *
 * Used for linear_scale and angular_scale. If the parameter has one
 * component, this value is copied as second and third components.
 *
 * @return True if the parameter was set correctly.
 */
bool ensureThreeComponents(std::vector<double>& param)
{
  if (param.size() == 0)
  {
    param.push_back(1);
    param.push_back(1);
    param.push_back(1);
    return True;
  }
  if (param.size() == 3)
  {
    return True;
  }
  if (param.size() == 1)
  {
    param.push_back(param[0]);
    param.push_back(param[0]);
    return True;
  }
  return False;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spacenav");

  ros::NodeHandle node_handle;
  ros::Publisher offset_pub = node_handle.advertise<geometry_msgs::Vector3>("spacenav/offset", 2);
  ros::Publisher rot_offset_pub = node_handle.advertise<geometry_msgs::Vector3>("spacenav/rot_offset", 2);
  ros::Publisher twist_pub = node_handle.advertise<geometry_msgs::Twist>("spacenav/twist", 2);
  ros::Publisher joy_pub = node_handle.advertise<sensor_msgs::Joy>("spacenav/joy", 2);

  // Used to scale joystick output to be in [-1, 1]. Estimated from data, and not necessarily correct.
  ros::NodeHandle private_nh("~");
  double full_scale;
  private_nh.param<double>("full_scale", full_scale, 512);
  if (full_scale < 1e-10)
  {
    full_scale = 512;
  }
  // Scale factors for the different axes. End output will be within [-scale, +scale], provided
  // full_scale normalizes to within [-1, 1].
  std::vector<double> linear_scale;
  std::vector<double> angular_scale;
  private_nh.param<std::vector<double> >("linear_scale", linear_scale, std::vector<double>(3, 1));
  private_nh.param<std::vector<double> >("angular_scale", angular_scale, std::vector<double>(3, 1));

  if (!ensureThreeComponents(linear_scale))
  {
    ROS_ERROR_STREAM("Parameter " << private_nh.getNamespace() << "/linear_scale must have either one or three components.");
    exit(EXIT_FAILURE);
  }
  if (!ensureThreeComponents(angular_scale))
  {
    ROS_ERROR_STREAM("Parameter " << private_nh.getNamespace() << "/angular_scale must have either one or three components.");
    exit(EXIT_FAILURE);
  }
  ROS_DEBUG("linear_scale: %.3f %.3f %.3f", linear_scale[0], linear_scale[1], linear_scale[2]);
  ROS_DEBUG("angular_scale: %.3f %.3f %.3f", angular_scale[0], angular_scale[1], angular_scale[2]);

  if (spnav_open() == -1)
  {
    ROS_ERROR("Could not open the space navigator device.  Did you remember to run spacenavd (as root)?");

    return 1;
  }

  // Parameters
  // The number of polls needed to be done before the device is considered "static"
  int static_count_threshold = 30;
  ros::param::get("~/static_count_threshold",static_count_threshold);

  // If true, the node will zero the output when the device is "static"
  bool zero_when_static = true;
  ros::param::get("~/zero_when_static",zero_when_static);

  // If the device is considered "static" and each trans, rot normed component
  // is below the deadband, it will output zeros in either rotation,
  // translation, or both.
  double static_trans_deadband = 0.1; 
  double static_rot_deadband = 0.1;
  ros::param::get("~/static_trans_deadband", static_trans_deadband);
  ros::param::get("~/static_rot_deadband", static_rot_deadband);

  sensor_msgs::Joy joystick_msg;
  joystick_msg.axes.resize(6);
  joystick_msg.buttons.resize(2);
  
  spnav_event sev;
  int no_motion_count = 0;
  bool motion_stale = false;
  double normed_vx = 0;
  double normed_vy = 0;
  double normed_vz = 0;
  double normed_wx = 0;
  double normed_wy = 0;
  double normed_wz = 0;
  while (node_handle.ok())
  {
    bool joy_stale = false;
    bool queue_empty = false;
    
    // Sleep when the queue is empty.
    // If the queue is empty 30 times in a row output zeros.
    // Output changes each time a button event happens, or when a motion
    // event happens and the queue is empty.
    joystick_msg.header.stamp = ros::Time().now();
    switch (spnav_poll_event(&sev))
    {
      case 0:
        queue_empty = true;
        if (++no_motion_count > static_count_threshold)
        {
          if ( zero_when_static || 
              ( fabs(normed_vx) < static_trans_deadband &&
                fabs(normed_vy) < static_trans_deadband &&
                fabs(normed_vz) < static_trans_deadband)
             )
          {
            normed_vx = normed_vy = normed_vz = 0;
          }

          if ( zero_when_static || 
              ( fabs(normed_wx) < static_rot_deadband &&
                fabs(normed_wy) < static_rot_deadband &&
                fabs(normed_wz) < static_rot_deadband )
             )
          {
            normed_wx = normed_wy = normed_wz = 0;
          }

          no_motion_count = 0;
          motion_stale = true;
        }
        break;

      case SPNAV_EVENT_MOTION:
        normed_vx = sev.motion.z / full_scale;
        normed_vy = -sev.motion.x / full_scale;
        normed_vz = sev.motion.y / full_scale;

        normed_wx = sev.motion.rz / full_scale;
        normed_wy = -sev.motion.rx / full_scale;
        normed_wz = sev.motion.ry / full_scale;

        motion_stale = true;
        break;
        
      case SPNAV_EVENT_BUTTON:
        //printf("type, press, bnum = <%d, %d, %d>\n", sev.button.type, sev.button.press, sev.button.bnum);
        joystick_msg.buttons[sev.button.bnum] = sev.button.press;

        joy_stale = true;
        break;

      default:
        ROS_WARN("Unknown message type in spacenav. This should never happen.");
        break;
    }
  
    if (motion_stale && (queue_empty || joy_stale))
    {
      // The offset and rot_offset are scaled.
      geometry_msgs::Vector3 offset_msg;
      offset_msg.x = normed_vx * linear_scale[0];
      offset_msg.y = normed_vy * linear_scale[1];
      offset_msg.z = normed_vz * linear_scale[2];
      offset_pub.publish(offset_msg);
      geometry_msgs::Vector3 rot_offset_msg;
      rot_offset_msg.x = normed_wx * angular_scale[0];
      rot_offset_msg.y = normed_wy * angular_scale[1];
      rot_offset_msg.z = normed_wz * angular_scale[2];
      rot_offset_pub.publish(rot_offset_msg);

      geometry_msgs::Twist twist_msg;
      twist_msg.linear = offset_msg;
      twist_msg.angular = rot_offset_msg;
      twist_pub.publish(twist_msg);

      // The joystick.axes are normalized within [-1, 1].
      joystick_msg.axes[0] = normed_vx;
      joystick_msg.axes[1] = normed_vy;
      joystick_msg.axes[2] = normed_vz;
      joystick_msg.axes[3] = normed_wx;
      joystick_msg.axes[4] = normed_wy;
      joystick_msg.axes[5] = normed_wz;

      no_motion_count = 0;
      motion_stale = false;
      joy_stale = true;
    }
  
    if (joy_stale)
    {
      joy_pub.publish(joystick_msg);
    }

    if (queue_empty) {
      usleep(1000);
    }
  }

  spnav_close();

  return 0;
}
