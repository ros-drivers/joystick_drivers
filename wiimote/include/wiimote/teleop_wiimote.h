/*
 * ROS Node for using a wiimote control unit to direct a robot.
 * Copyright (c) 2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

/*
 * Initial C++ implementation by
 *   Mark Horn <mark.d.horn@intel.com>
 *
 * Revisions:
 *
 */

#pragma once
#ifndef WIIMOTE_TELEOP_WIIMOTE_H
#define WIIMOTE_TELEOP_WIIMOTE_H

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "wiimote/State.h"

#include <math.h>

// Sane defaults based on the TurtleBot
// TurtleBot maximum speed documented at 25.6"/second ~= 0.65024 m/s
#define DEFAULT_MAX_LINEAR_X  0.65024  // m/s
// TurtleBot maximum angular speed is documented at 180 degrees Pi / second
#define DEFAULT_MAX_ANGULAR_Z  M_PI    // rad/s

#define DEFAULT_PERCENT_LINEAR_THROTTLE 0.75
#define DEFAULT_PERCENT_ANGULAR_THROTTLE 0.75

class TeleopWiimote
{
public:
  TeleopWiimote();

private:
  void rumbleFeedback(int useconds);
  void setLEDFeedback(double value);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void wiimoteStateCallback(const wiimote::State::ConstPtr& wiistate);

  double linear_x_max_velocity_;   // m/s
  double linear_x_min_velocity_;   // m/s
  double angular_z_max_velocity_;  // rad/s
  double angular_z_min_velocity_;  // rad/s

  double percent_linear_throttle_;   // 0.0 - 1.0 (1.0 = 100%)
  double percent_angular_throttle_;  // 0.0 - 1.0 (1.0 = 100%)

  ros::Publisher vel_pub_;
  ros::Publisher joy_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber wiimote_sub_;

  bool dpad_in_use_ = false;
  bool njoy_in_use_ = false;
};

#endif  // WIIMOTE_TELEOP_WIIMOTE_H
