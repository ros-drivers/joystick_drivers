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

#include "wiimote/teleop_wiimote.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JoyFeedbackArray.h"

#include <string>

TeleopWiimote::TeleopWiimote()
{
  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh;
  std::string base_name;

  if (nh_private.hasParam("base"))
  {
    if (nh_private.getParam("base", base_name))
    {
      ROS_INFO("User namespace '%s' for robot limits.", base_name.c_str());
    }
  }
  base_name += "/";

  if (!nh.getParam(base_name + "linear/x/max_velocity", linear_x_max_velocity_))
  {
    linear_x_max_velocity_ = DEFAULT_MAX_LINEAR_X;
    ROS_INFO("Defaulting to maximum linear x: %f", linear_x_max_velocity_);
  }
  else
  {
    ROS_INFO("Maximum linear x: %f", linear_x_max_velocity_);
  }
  if (!nh.getParam(base_name + "linear/x/min_velocity", linear_x_min_velocity_))
  {
    linear_x_min_velocity_ = -1.0 * DEFAULT_MAX_LINEAR_X;
    ROS_INFO("Defaulting to minimum linear x: %f", linear_x_min_velocity_);
  }
  else
  {
    ROS_INFO("Minimum linear x: %f", linear_x_min_velocity_);
  }

  if (!nh.getParam(base_name + "angular/z/max_velocity", angular_z_max_velocity_))
  {
    angular_z_max_velocity_ = DEFAULT_MAX_ANGULAR_Z;
    ROS_INFO("Defaulting to maximum angular z: %f", angular_z_max_velocity_);
  }
  else
  {
    ROS_INFO("Maximum angular z: %f", angular_z_max_velocity_);
  }
  if (!nh.getParam(base_name + "angular/z/min_velocity", angular_z_min_velocity_))
  {
    angular_z_min_velocity_ = -1.0 * DEFAULT_MAX_ANGULAR_Z;
    ROS_INFO("Defaulting to minimum angular z: %f", angular_z_min_velocity_);
  }
  else
  {
    ROS_INFO("Minimum angular z: %f", angular_z_min_velocity_);
  }

  // Percent throttle limiter
  if (nh_private.hasParam("linear/x/throttle_percent"))
  {
    if (!nh_private.getParam("linear/x/throttle_percent", percent_linear_throttle_))
    {
      ROS_WARN("Failed to get linear x throttle percent; using %3.0f", DEFAULT_PERCENT_LINEAR_THROTTLE * 100.0);
      percent_linear_throttle_ = DEFAULT_PERCENT_LINEAR_THROTTLE;
    }
  }
  else
  {
    percent_linear_throttle_ = DEFAULT_PERCENT_LINEAR_THROTTLE;
  }
  nh_private.setParam("linear/x/throttle_percent", percent_linear_throttle_);
  ROS_INFO("Linear X Throttle Percent: %3.0f", percent_linear_throttle_ * 100.0);

  if (nh_private.hasParam("angular/z/throttle_percent"))
  {
    if (!nh_private.getParam("angular/z/throttle_percent", percent_angular_throttle_))
    {
      ROS_WARN("Failed to get angular z throttle percent; using %3.0f", DEFAULT_PERCENT_ANGULAR_THROTTLE * 100.0);
      percent_angular_throttle_ = DEFAULT_PERCENT_ANGULAR_THROTTLE;
    }
  }
  else
  {
    percent_angular_throttle_ = DEFAULT_PERCENT_ANGULAR_THROTTLE;
  }
  nh_private.setParam("angular/x/throttle_percent", percent_angular_throttle_);
  ROS_INFO("Angular Z Throttle Percent: %3.0f", percent_angular_throttle_ * 100.0);

  vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  joy_pub_ = nh.advertise<sensor_msgs::JoyFeedbackArray>("joy/set_feedback", 1);

  joy_sub_ = nh.subscribe<sensor_msgs::Joy>("wiimote/nunchuk", 10, &TeleopWiimote::joyCallback, this);
  wiimote_sub_ = nh.subscribe<wiimote::State>("wiimote/state", 10, &TeleopWiimote::wiimoteStateCallback, this);

  dpad_in_use_ = false;
  njoy_in_use_ = false;
}

void TeleopWiimote::setLEDFeedback(double value)
{
  sensor_msgs::JoyFeedbackArray joy_feedback_array;
  sensor_msgs::JoyFeedback fb_led0;
  sensor_msgs::JoyFeedback fb_led1;
  sensor_msgs::JoyFeedback fb_led2;
  sensor_msgs::JoyFeedback fb_led3;

  fb_led0.type = sensor_msgs::JoyFeedback::TYPE_LED;
  fb_led0.id = 0;
  fb_led0.intensity = 0.0;
  fb_led1.type = sensor_msgs::JoyFeedback::TYPE_LED;
  fb_led1.id = 1;
  fb_led1.intensity = 0.0;
  fb_led2.type = sensor_msgs::JoyFeedback::TYPE_LED;
  fb_led2.id = 2;
  fb_led2.intensity = 0.0;
  fb_led3.type = sensor_msgs::JoyFeedback::TYPE_LED;
  fb_led3.id = 3;
  fb_led3.intensity = 0.0;

  if (value > 10.0)
  {
    fb_led0.intensity = 1.0;
  }
  if (value > 35.0)
  {
    fb_led1.intensity = 1.0;
  }
  if (value > 60.0)
  {
    fb_led2.intensity = 1.0;
  }
  if (value > 85.0)
  {
    fb_led3.intensity = 1.0;
  }

  joy_feedback_array.array.push_back(fb_led0);
  joy_feedback_array.array.push_back(fb_led1);
  joy_feedback_array.array.push_back(fb_led2);
  joy_feedback_array.array.push_back(fb_led3);

  joy_pub_.publish(joy_feedback_array);
}

void TeleopWiimote::rumbleFeedback(int useconds)
{
  sensor_msgs::JoyFeedbackArray joy_feedback_array;
  sensor_msgs::JoyFeedback fb_rumble;

  fb_rumble.type = sensor_msgs::JoyFeedback::TYPE_RUMBLE;
  fb_rumble.id = 0;

  fb_rumble.intensity = 1.0;

  joy_feedback_array.array.push_back(fb_rumble);

  joy_pub_.publish(joy_feedback_array);
  usleep(useconds);
  fb_rumble.intensity = 0.0;

  joy_feedback_array.array.push_back(fb_rumble);

  joy_pub_.publish(joy_feedback_array);
}

void TeleopWiimote::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;

  static const int MSG_BTN_Z     = 0;
  static const int MSG_BTN_C     = 1;

  if (dpad_in_use_)
  {
    return;
  }

  float x = joy->axes[0];
  float y = joy->axes[1];
  float const abs_error = 0.000001;

  if (fabs(x) > abs_error || fabs(y) > abs_error)
  {
    njoy_in_use_ = true;

    float boost = 1.0;

    ROS_INFO("nunchuk: x: %f, y: %f", x, y);

    if (joy->buttons[MSG_BTN_Z] ||
        joy->buttons[MSG_BTN_C])
    {
      ROS_INFO("buttons[]: Z: %d, C: %d",
          joy->buttons[MSG_BTN_Z],
          joy->buttons[MSG_BTN_C]);

      // Z-Button is thrusters on!
      if (joy->buttons[MSG_BTN_Z])
      {
        boost = 2.0;
      }

      // C-Button is easy does it.
      if (joy->buttons[MSG_BTN_C])
      {
        boost = 0.25;
      }
    }

    if (y >= 0.0)
    {
      vel.linear.x = fmin((y * boost * (linear_x_max_velocity_ * percent_linear_throttle_)),
          linear_x_max_velocity_);

      if (x >= 0.0)
      {
        vel.angular.z = fmin((x * boost * (angular_z_max_velocity_ * percent_angular_throttle_)),
            angular_z_max_velocity_);
      }
      else
      {
        vel.angular.z = fmax((fabs(x) * boost * (angular_z_min_velocity_ * percent_angular_throttle_)),
            angular_z_min_velocity_);
      }
    }
    else
    {
      vel.linear.x = fmax((fabs(y) * boost * (linear_x_min_velocity_ * percent_linear_throttle_)),
          linear_x_min_velocity_);

      if (x > 0.0)
      {
        vel.angular.z = fmax((x * boost * (angular_z_min_velocity_ * percent_angular_throttle_)),
            angular_z_min_velocity_);
      }
      else
      {
        vel.angular.z = fmin((fabs(x) * boost * (angular_z_max_velocity_ * percent_angular_throttle_)),
            angular_z_max_velocity_);
      }
    }

    // In order to spin-in-place left or right, we need full angular with NO linear component.
    // To enable this, we will publish no linear motion if nunchuk joystick Y value is
    // "really small" as the joy stick isn't 100 accurate.
    if (fabs(y) < 0.01)
    {
      vel.linear.x = 0;
    }

    vel_pub_.publish(vel);
  }
  else
  {
    if (njoy_in_use_)
    {
      vel_pub_.publish(vel);

      njoy_in_use_ = false;
    }
  }
}
void TeleopWiimote::wiimoteStateCallback(const wiimote::State::ConstPtr& wiistate)
{
  ros::NodeHandle nh_private("~");
  geometry_msgs::Twist vel;

  static const int MSG_BTN_1     = 0;
  static const int MSG_BTN_2     = 1;
  static const int MSG_BTN_PLUS  = 2;
  static const int MSG_BTN_MINUS = 3;
  static const int MSG_BTN_A     = 4;
  static const int MSG_BTN_B     = 5;
  static const int MSG_BTN_UP    = 6;
  static const int MSG_BTN_DOWN  = 7;
  static const int MSG_BTN_LEFT  = 8;
  static const int MSG_BTN_RIGHT = 9;
  static const int MSG_BTN_HOME  = 10;

  static bool one_depressed = false;
  static bool two_depressed = false;
  static bool plus_depressed = false;
  static bool minus_depressed = false;
  static bool home_depressed = false;


  // 1-Button used to set the amount of Linear Throttle
  // Pressing the button show approx setting level on the
  // Wiimote LEDs (see setLEDFeedback for levels).
  // Wiimote uses a short Rumble when the minimum or
  // maximum is reached.
  // +-Button increases; --Button decreases while hold 1-Button
  if (wiistate->buttons[MSG_BTN_1])
  {
    if (wiistate->buttons[MSG_BTN_PLUS])
    {
      if (!plus_depressed)
      {
        percent_linear_throttle_ += 0.05;
        if (percent_linear_throttle_ >= 1.0)
        {
          rumbleFeedback(100000);
        }
        percent_linear_throttle_ = fmin(percent_linear_throttle_, 1.0);
        plus_depressed = true;

        setLEDFeedback(percent_linear_throttle_ * 100.0);

        nh_private.setParam("linear/x/throttle_percent", percent_linear_throttle_);
        ROS_INFO("Linear X Throttle Percent: %3.0f", percent_linear_throttle_ * 100.0);
      }
    }
    else
    {
      plus_depressed = false;

      if (wiistate->buttons[MSG_BTN_MINUS])
      {
        if (!minus_depressed)
        {
          percent_linear_throttle_ -= 0.05;
          if (percent_linear_throttle_ <= 0.1)
          {
            rumbleFeedback(100000);
          }
          percent_linear_throttle_ = fmax(percent_linear_throttle_, 0.1);
          minus_depressed = true;

          setLEDFeedback(percent_linear_throttle_ * 100.0);

          nh_private.setParam("linear/x/throttle_percent", percent_linear_throttle_);
          ROS_INFO("Linear X Throttle Percent: %3.0f", percent_linear_throttle_ * 100.0);
        }
      }
      else
      {
        minus_depressed = false;
      }
    }

    if (!one_depressed)
    {
      setLEDFeedback(percent_linear_throttle_ * 100.0);

      nh_private.setParam("linear/x/throttle_percent", percent_linear_throttle_);
      ROS_INFO("Linear X Throttle Percent: %3.0f", percent_linear_throttle_ * 100.0);

      one_depressed = true;
    }
  }
  // 2-Button used to set the amount of Angular Throttle
  // Same function and feedbacks as 1-Button (see above)
  else if (wiistate->buttons[MSG_BTN_2])
  {
    if (wiistate->buttons[MSG_BTN_PLUS])
    {
      if (!plus_depressed)
      {
        percent_angular_throttle_ += 0.05;
        if (percent_angular_throttle_ >= 1.0)
        {
          rumbleFeedback(100000);
        }
        percent_angular_throttle_ = fmin(percent_angular_throttle_, 1.0);
        plus_depressed = true;

        setLEDFeedback(percent_angular_throttle_ * 100.0);

        nh_private.setParam("angular/x/throttle_percent", percent_angular_throttle_);
        ROS_INFO("Angular Z Throttle Percent: %3.0f", percent_angular_throttle_ * 100.0);
      }
    }
    else
    {
      plus_depressed = false;

      if (wiistate->buttons[MSG_BTN_MINUS])
      {
        if (!minus_depressed)
        {
          percent_angular_throttle_ -= 0.05;
          if (percent_angular_throttle_ <= 0.1)
          {
            rumbleFeedback(100000);
          }
          percent_angular_throttle_ = fmax(percent_angular_throttle_, 0.1);
          minus_depressed = true;

          setLEDFeedback(percent_angular_throttle_ * 100.0);

          nh_private.setParam("angular/x/throttle_percent", percent_angular_throttle_);
          ROS_INFO("Angular Z Throttle Percent: %3.0f", percent_angular_throttle_ * 100.0);
        }
      }
      else
      {
        minus_depressed = false;
      }
    }

    if (!two_depressed)
    {
      setLEDFeedback(percent_angular_throttle_ * 100.0);

      nh_private.setParam("angular/x/throttle_percent", percent_angular_throttle_);
      ROS_INFO("Angular Z Throttle Percent: %3.0f", percent_angular_throttle_ * 100.0);

      two_depressed = true;
    }
  }
  else
  {
    if (one_depressed || two_depressed)
    {
      setLEDFeedback(0.0);
    }

    one_depressed = false;
    two_depressed = false;

    // Home-Button used the Wiimote LEDs (see setLEDFeedback for levels).
    // to show the approx battery leve of the Wiimote.
    // Only works if the 1-Button or 2-Button are not in use.
    if (wiistate->buttons[MSG_BTN_HOME])
    {
      if (!home_depressed)
      {
        ROS_INFO("Battery[]: raw: %f, percent: %f", wiistate->raw_battery, wiistate->percent_battery);
        setLEDFeedback(wiistate->percent_battery);
        home_depressed = true;
      }
    }
    else
    {
      if (home_depressed)
      {
        setLEDFeedback(0.0);
      }

      home_depressed = false;
    }
  }

  if (!njoy_in_use_ &&
      (wiistate->buttons[MSG_BTN_RIGHT] ||
       wiistate->buttons[MSG_BTN_LEFT] ||
       wiistate->buttons[MSG_BTN_UP] ||
       wiistate->buttons[MSG_BTN_DOWN]))
  {
    dpad_in_use_ = true;

    ROS_INFO("buttons[]: Right: %d, Left: %d, Up: %d, Down: %d, A: %d, B: %d",
        wiistate->buttons[MSG_BTN_RIGHT],
        wiistate->buttons[MSG_BTN_LEFT],
        wiistate->buttons[MSG_BTN_UP],
        wiistate->buttons[MSG_BTN_DOWN],
        wiistate->buttons[MSG_BTN_A],
        wiistate->buttons[MSG_BTN_B]);

    float boost = 1.0;

    // B-Button is thrusters on!
    if (wiistate->buttons[MSG_BTN_B])
    {
      boost = 2.0;
    }

    // A-Button is easy does it.
    if (wiistate->buttons[MSG_BTN_A])
    {
      boost = 0.25;
    }

    if (wiistate->buttons[MSG_BTN_UP])
    {
      vel.linear.x = fmin((boost * (linear_x_max_velocity_ * percent_linear_throttle_)),
          linear_x_max_velocity_);
    }
    else if (wiistate->buttons[MSG_BTN_DOWN])
    {
      vel.linear.x = fmax((boost * (linear_x_min_velocity_ * percent_linear_throttle_)),
          linear_x_min_velocity_);
    }

    if (wiistate->buttons[MSG_BTN_LEFT])
    {
      vel.angular.z = fmin((boost * (angular_z_max_velocity_ * percent_angular_throttle_)),
          angular_z_max_velocity_);
    }
    else if (wiistate->buttons[MSG_BTN_RIGHT])
    {
      vel.angular.z = fmax((boost * (angular_z_min_velocity_ * percent_angular_throttle_)),
          angular_z_min_velocity_);
    }

    vel_pub_.publish(vel);
  }
  else
  {
    if (dpad_in_use_)
    {
      vel_pub_.publish(vel);
      dpad_in_use_ = false;
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_wiimote");
  TeleopWiimote teleop_wiimote;

  ros::spin();
}
