/*
 * ROS Node for interfacing with a wiimote control unit.
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
 * This code is based on the original Python implementation
 * created by Andreas Paepcke and Melonee Wise
 *  Andreas Paepcke <paepcke@anw.willowgarage.com>
 * with contributions by
 *  David Lu <davidlu@wustl.edu>
 *  Miguel Angel Julian Aguilar <miguel.angel@thecorpora.com>
 * See https://github.com/ros-drivers/joystick_drivers/tree/indigo-devel/wiimote
 * for details and history.
 *
 * This C++ implementation used the functionality of the existing
 * Python code as the feature requirements.
 */

/*
 * Initial C++ implementation by
 *   Mark Horn <mark.d.horn@intel.com>
 *
 * The C++ implementation was designed with focus on reduced resource consumption.
 *
 * Differences from Python implementation:
 * - Both "/wiimote/nunchuk" and "/wiimote/classic" topics are only published
 *   if the Nunchuk or Classic Controller are connected to the wiimote respectively.
 * - Wiimote data is only polled from the controller if the data is required
 *   to publish for a topic which has at least one subscriber.
 *
 * Revisions:
 *
 */

#include "wiimote/wiimote_controller.h"

#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"

#include "wiimote/State.h"
#include "wiimote/IrSourceInfo.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <signal.h>

#include <time.h>
#include <math.h>

#include <vector>
#include <string>

WiimoteNode::WiimoteNode()
{
  joy_pub_ = nh_.advertise<sensor_msgs::Joy>("/joy", 1);
  imu_data_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data", 1);
  wiimote_state_pub_ = nh_.advertise<wiimote::State>("/wiimote/state", 1);
  /**
   * Optional Publications -- only advertise if the hardware is connected
   * This is done in the main loop in the ::publish method
   * wiimote_nunchuk_pub_ = nh_.advertise<sensor_msgs::Joy>("/wiimote/nunchuk", 1);
   * wiimote_classic_pub_ = nh_.advertise<sensor_msgs::Joy>("/wiimote/classic", 1);
   **/
  imu_is_calibrated_pub_ = nh_.advertise<std_msgs::Bool>("/imu/is_calibrated", 1, true);

  joy_set_feedback_sub_ = nh_.subscribe<sensor_msgs::JoyFeedbackArray>("/joy/set_feedback", 10,
      &WiimoteNode::joySetFeedbackCallback, this);

  imu_calibrate_srv_ = nh_.advertiseService("/imu/calibrate", &WiimoteNode::serviceImuCalibrateCallback, this);

  // Initialize with the ANY Bluetooth Address
  setBluetoothAddr("00:00:00:00:00:00");

  wiimote_ = nullptr;

  initializeWiimoteState();

  state_secs_ = 0;
  state_nsecs_ = 0;

  // Setup the Wii Error Handler
  wiimote_c::cwiid_set_err(cwiidErrorCallback);

  report_mode_ = 0;

  wiimote_calibrated_ = false;

  resetMotionPlusState();
  resetNunchukState();
  resetClassicState();
  nunchuk_failed_calibration_ = false;
}

WiimoteNode::~WiimoteNode()
{
}

void WiimoteNode::initializeWiimoteState()
{
  wiimote_state_.rpt_mode = 0;
  wiimote_state_.led = 0;
  wiimote_state_.rumble = 0;
  wiimote_state_.battery = 0;
  wiimote_state_.buttons = 0;
  wiimote_state_.acc[0] = 0;
  wiimote_state_.acc[1] = 0;
  wiimote_state_.acc[2] = 0;

  wiimote_state_.ext_type = wiimote_c::CWIID_EXT_NONE;
  wiimote_state_.error = wiimote_c::CWIID_ERROR_NONE;

  wiimote_state_.ir_src[0].valid = 0;
  wiimote_state_.ir_src[1].valid = 0;
  wiimote_state_.ir_src[2].valid = 0;
  wiimote_state_.ir_src[3].valid = 0;
  wiimote_state_.ir_src[0].pos[0] = 0; wiimote_state_.ir_src[0].pos[1] = 0;
  wiimote_state_.ir_src[1].pos[0] = 0; wiimote_state_.ir_src[1].pos[1] = 0;
  wiimote_state_.ir_src[2].pos[0] = 0; wiimote_state_.ir_src[2].pos[1] = 0;
  wiimote_state_.ir_src[3].pos[0] = 0; wiimote_state_.ir_src[3].pos[1] = 0;
  wiimote_state_.ir_src[0].size = 0;
  wiimote_state_.ir_src[1].size = 0;
  wiimote_state_.ir_src[2].size = 0;
  wiimote_state_.ir_src[3].size = 0;
}

char *WiimoteNode::getBluetoothAddr()
{
  return batostr(&bt_device_addr_);
}
void WiimoteNode::setBluetoothAddr(const char *bt_str)
{
  str2ba(bt_str, &bt_device_addr_);
}

bool WiimoteNode::pairWiimote(int flags = 0, int timeout = 5)
{
  bool status = true;

  ROS_INFO("Put Wiimote in discoverable mode now (press 1+2)...");
  if (timeout == -1)
    ROS_INFO("Searching indefinitely...");
  else
    ROS_INFO("Timeout in about %d seconds if not paired.", timeout);

  if (!(wiimote_ = wiimote_c::cwiid_open_timeout(&bt_device_addr_, flags, timeout)))
  {
    ROS_ERROR("Unable to connect to wiimote");
    status = false;
  }
  else
  {
    // Give the hardware time to zero the accelerometer and gyro after pairing
    // Ensure we are getting valid data before using
    sleep(1);

    checkFactoryCalibrationData();

    if (!wiimote_calibrated_)
    {
      ROS_ERROR("Wiimote not usable due to calibration failure.");
      status = false;
    }
  }

  return status;
}
int WiimoteNode::unpairWiimote()
{
  return wiimote_c::cwiid_close(wiimote_);
}

void WiimoteNode::checkFactoryCalibrationData()
{
  bool result = true;

  if (wiimote_c::cwiid_get_acc_cal(wiimote_, wiimote_c::CWIID_EXT_NONE, &wiimote_cal_) != 0)
  {
    if (wiimote_calibrated_)
    {
      ROS_WARN("Failed to read current Wiimote calibration data; proceeding with previous data");
    }
    else
    {
      ROS_ERROR("Failed to read Wiimote factory calibration data");
      result = false;
    }
  }
  else
  {
    // If any calibration point is zero; we fail
    if (!(wiimote_cal_.zero[CWIID_X] && wiimote_cal_.zero[CWIID_Y] && wiimote_cal_.zero[CWIID_Z] &&
        wiimote_cal_.one[CWIID_X] && wiimote_cal_.one[CWIID_Y] && wiimote_cal_.one[CWIID_Z]))
    {
      ROS_ERROR("Some Wiimote factory calibration data is missing; calibration failed");
      ROS_ERROR("Wiimote Cal:: zero[x]:%d, zero[y]:%d, zero[z]:%d,\n\tone[x]:%d, one[y]:%d, one[z]:%d",
          wiimote_cal_.zero[CWIID_X], wiimote_cal_.zero[CWIID_Y], wiimote_cal_.zero[CWIID_Z],
          wiimote_cal_.one[CWIID_X], wiimote_cal_.one[CWIID_Y], wiimote_cal_.one[CWIID_Z]);

      result = false;
    }
    else
    {
      wiimote_calibrated_ = true;
      ROS_DEBUG("Wiimote Cal:: zero[x]:%d, zero[y]:%d, zero[z]:%d,\n\tone[x]:%d, one[y]:%d, one[z]:%d",
          wiimote_cal_.zero[CWIID_X], wiimote_cal_.zero[CWIID_Y], wiimote_cal_.zero[CWIID_Z],
          wiimote_cal_.one[CWIID_X], wiimote_cal_.one[CWIID_Y], wiimote_cal_.one[CWIID_Z]);
    }
  }

  if (!getStateSample())
  {
    ROS_WARN("Could not read Wiimote state; nunchuk may not be calibrated if present.");
  }
  else
  {
    if (isPresentNunchuk())
    {
      if (wiimote_c::cwiid_get_acc_cal(wiimote_, wiimote_c::CWIID_EXT_NUNCHUK, &nunchuk_cal_) != 0)
      {
        if (nunchuk_calibrated_)
        {
          ROS_WARN("Failed to read current Nunchuk calibration data; proceeding with previous data");
        }
        else
        {
          ROS_ERROR("Failed to read Nunchuk factory calibration data");
          result = false;
          nunchuk_failed_calibration_ = true;
        }
      }
      else
      {
        // If any calibration point is zero; we fail
        if (!(nunchuk_cal_.zero[CWIID_X] && nunchuk_cal_.zero[CWIID_Y] && nunchuk_cal_.zero[CWIID_Z] &&
            nunchuk_cal_.one[CWIID_X] && nunchuk_cal_.one[CWIID_Y] && nunchuk_cal_.one[CWIID_Z]))
        {
          ROS_ERROR("Some Nunchuk factory calibration data is missing; calibration failed");
          ROS_ERROR("Nunchuk Cal:: zero[x]:%d, zero[y]:%d, zero[z]:%d,\n\tone[x]:%d, one[y]:%d, one[z]:%d",
              nunchuk_cal_.zero[CWIID_X], nunchuk_cal_.zero[CWIID_Y], nunchuk_cal_.zero[CWIID_Z],
              nunchuk_cal_.one[CWIID_X], nunchuk_cal_.one[CWIID_Y], nunchuk_cal_.one[CWIID_Z]);
          result = false;
          nunchuk_failed_calibration_ = true;
        }
        else
        {
          nunchuk_calibrated_ = true;
          ROS_DEBUG("Nunchuk Cal:: zero[x]:%d, zero[y]:%d, zero[z]:%d,\n\tone[x]:%d, one[y]:%d, one[z]:%d",
              nunchuk_cal_.zero[CWIID_X], nunchuk_cal_.zero[CWIID_Y], nunchuk_cal_.zero[CWIID_Z],
              nunchuk_cal_.one[CWIID_X], nunchuk_cal_.one[CWIID_Y], nunchuk_cal_.one[CWIID_Z]);
        }
      }
    }
  }

  if (wiimote_calibrated_)
  {
    // Save the current reporting mode
    uint8_t save_report_mode = wiimote_state_.rpt_mode;

    // Need to ensure we are collecting accelerometer
    uint8_t new_report_mode = save_report_mode | (CWIID_RPT_ACC | CWIID_RPT_EXT);

    if (new_report_mode != save_report_mode)
    {
      setReportMode(new_report_mode);
    }

    ROS_INFO("Collecting additional calibration data; keep wiimote stationary...");

    StatVector3d linear_acceleration_stat_old = linear_acceleration_stat_;
    linear_acceleration_stat_.clear();
    StatVector3d angular_velocity_stat_old = angular_velocity_stat_;
    angular_velocity_stat_.clear();

    bool failed = false;
    bool data_complete = false;
    int wiimote_data_points = 0;
    int motionplus_data_points = 0;

    while (!failed && !data_complete)
    {
      if (getStateSample())
      {
        if (wiimote_data_points < COVARIANCE_DATA_POINTS_)
        {
          linear_acceleration_stat_.addData(
              wiimote_state_.acc[CWIID_X],
              wiimote_state_.acc[CWIID_Y],
              wiimote_state_.acc[CWIID_Z]);

          ++wiimote_data_points;
        }

        if (isPresentMotionplus())
        {
          if (motionplus_data_points < COVARIANCE_DATA_POINTS_)
          {
            // ROS_DEBUG("New MotionPlus data :%03d: PHI: %04d, THETA: %04d, PSI: %04d", motionplus_data_points,
            //     wiimote_state_.ext.motionplus.angle_rate[CWIID_PHI],
            //     wiimote_state_.ext.motionplus.angle_rate[CWIID_THETA],
            //     wiimote_state_.ext.motionplus.angle_rate[CWIID_PSI]);
            angular_velocity_stat_.addData(
                wiimote_state_.ext.motionplus.angle_rate[CWIID_PHI],
                wiimote_state_.ext.motionplus.angle_rate[CWIID_THETA],
                wiimote_state_.ext.motionplus.angle_rate[CWIID_PSI]);

            ++motionplus_data_points;
          }
        }
      }
      else
      {
        failed = true;
      }

      if (wiimote_data_points >= COVARIANCE_DATA_POINTS_)
      {
        if (!isPresentMotionplus())
        {
          data_complete = true;
        }
        else
        {
          if (motionplus_data_points >= COVARIANCE_DATA_POINTS_)
          {
            data_complete = true;
          }
        }
      }
    }

    if (!failed)
    {
      ROS_DEBUG("Calculating calibration data...");

      // Check the standard deviations > 1.0
      TVectorDouble stddev = linear_acceleration_stat_.getStandardDeviationRaw();
      bool is_bad_cal = false;
      std::for_each(std::begin(stddev), std::end(stddev), [&](const double d)  // NOLINT(build/c++11)
      {
        if (d > 1.0)
        {
          is_bad_cal = true;
          ROS_DEBUG("Wiimote standard deviation > 1.0");
        }
      });  // NOLINT(whitespace/braces)

      if (!is_bad_cal)
      {
        TVectorDouble variance = linear_acceleration_stat_.getVarianceScaled(EARTH_GRAVITY_);

        ROS_DEBUG("Variance Scaled x: %lf, y: %lf, z: %lf", variance.at(CWIID_X),
            variance.at(CWIID_Y), variance.at(CWIID_Z));

        linear_acceleration_covariance_[0] = variance.at(CWIID_X);
        linear_acceleration_covariance_[1] = 0.0;
        linear_acceleration_covariance_[2] = 0.0;

        linear_acceleration_covariance_[3] = 0.0;
        linear_acceleration_covariance_[4] = variance.at(CWIID_Y);
        linear_acceleration_covariance_[5] = 0.0;

        linear_acceleration_covariance_[6] = 0.0;
        linear_acceleration_covariance_[7] = 0.0;
        linear_acceleration_covariance_[8] = variance.at(CWIID_Z);
      }
      else
      {
        ROS_ERROR("Failed calibration; using questionable data for linear acceleration");

        linear_acceleration_stat_ = linear_acceleration_stat_old;
        angular_velocity_stat_ = angular_velocity_stat_old;

        result = false;
      }

      if (angular_velocity_stat_.size() == COVARIANCE_DATA_POINTS_)
      {
        // Check the standard deviations > 50.0
        TVectorDouble gyro_stddev = angular_velocity_stat_.getStandardDeviationRaw();
        std::for_each(std::begin(gyro_stddev), std::end(gyro_stddev), [&](const double d)  // NOLINT(build/c++11)
        {
          if (d > 50.0)
          {
            is_bad_cal = true;
            ROS_DEBUG("MotionPlus standard deviation > 50");
          }
        });  // NOLINT(whitespace/braces)

        if (!is_bad_cal)
        {
          TVectorDouble gyro_variance = angular_velocity_stat_.getVarianceScaled(GYRO_SCALE_FACTOR_);

          ROS_DEBUG("Gyro Variance Scaled x: %lf, y: %lf, z: %lf", gyro_variance.at(CWIID_PHI),
              gyro_variance.at(CWIID_THETA), gyro_variance.at(CWIID_PSI));

          angular_velocity_covariance_[0] = gyro_variance.at(CWIID_PHI);
          angular_velocity_covariance_[1] = 0.0;
          angular_velocity_covariance_[2] = 0.0;

          angular_velocity_covariance_[3] = 0.0;
          angular_velocity_covariance_[4] = gyro_variance.at(CWIID_THETA);
          angular_velocity_covariance_[5] = 0.0;

          angular_velocity_covariance_[6] = 0.0;
          angular_velocity_covariance_[7] = 0.0;
          angular_velocity_covariance_[8] = gyro_variance.at(CWIID_PSI);
        }
        else
        {
          ROS_ERROR("Failed calibration; using questionable data for angular velocity");

          angular_velocity_stat_ = angular_velocity_stat_old;

          result = false;
        }
      }
      else
      {
        resetMotionPlusState();
      }
    }

    if (failed)
    {
      ROS_ERROR("Failed calibration; using questionable data");
      result = false;
    }
    else
    {
      struct timespec state_tv;

      if (clock_gettime(CLOCK_REALTIME, &state_tv) == 0)
      {
        calibration_time_ = ros::Time::now();
      }
      else
      {
        ROS_WARN("Could not update calibration time.");
      }
    }

    // Restore the pre-existing reporting mode
    if (new_report_mode != save_report_mode)
    {
      setReportMode(save_report_mode);
    }
  }

  // Publish the initial calibration state
  std_msgs::Bool imu_is_calibrated_data;
  imu_is_calibrated_data.data = result;
  imu_is_calibrated_pub_.publish(imu_is_calibrated_data);
}

void WiimoteNode::resetMotionPlusState()
{
  // If no gyro is attached to the Wiimote then we signal
  // the invalidity of angular rate with a covariance matrix
  // whose first element is -1:
  angular_velocity_covariance_[0] = -1.0;
  angular_velocity_covariance_[1] = 0.0;
  angular_velocity_covariance_[2] = 0.0;

  angular_velocity_covariance_[3] = 0.0;
  angular_velocity_covariance_[4] = 0.0;
  angular_velocity_covariance_[5] = 0.0;

  angular_velocity_covariance_[6] = 0.0;
  angular_velocity_covariance_[7] = 0.0;
  angular_velocity_covariance_[8] = 0.0;
}

void WiimoteNode::resetNunchukState()
{
  nunchuk_calibrated_ = false;

  nunchuk_stick_calibrated_ = false;
  nunchuk_stick_center_[CWIID_X] = JOYSTICK_NUNCHUK_DEFAULT_CENTER_;
  nunchuk_stick_center_[CWIID_Y] = JOYSTICK_NUNCHUK_DEFAULT_CENTER_;
  nunchuk_stick_max_[CWIID_X] = JOYSTICK_NUNCHUK_20PERCENT_MAX_;
  nunchuk_stick_max_[CWIID_Y] = JOYSTICK_NUNCHUK_20PERCENT_MAX_;
  nunchuk_stick_min_[CWIID_X] = JOYSTICK_NUNCHUK_20PERCENT_MIN_;
  nunchuk_stick_min_[CWIID_Y] = JOYSTICK_NUNCHUK_20PERCENT_MIN_;
}

void WiimoteNode::resetClassicState()
{
  classic_stick_left_calibrated_ = false;
  classic_stick_right_calibrated_ = false;
  classic_stick_left_center_[CWIID_X] = JOYSTICK_CLASSIC_LEFT_DEFAULT_CENTER_;
  classic_stick_left_center_[CWIID_Y] = JOYSTICK_CLASSIC_LEFT_DEFAULT_CENTER_;
  classic_stick_right_center_[CWIID_X] = JOYSTICK_CLASSIC_RIGHT_DEFAULT_CENTER_;
  classic_stick_right_center_[CWIID_Y] = JOYSTICK_CLASSIC_RIGHT_DEFAULT_CENTER_;
  classic_stick_left_max_[CWIID_X] = JOYSTICK_CLASSIC_LEFT_20PERCENT_MAX_;
  classic_stick_left_max_[CWIID_Y] = JOYSTICK_CLASSIC_LEFT_20PERCENT_MAX_;
  classic_stick_left_min_[CWIID_X] = JOYSTICK_CLASSIC_LEFT_20PERCENT_MIN_;
  classic_stick_left_min_[CWIID_Y] = JOYSTICK_CLASSIC_LEFT_20PERCENT_MIN_;
  classic_stick_right_max_[CWIID_X] = JOYSTICK_CLASSIC_RIGHT_20PERCENT_MAX_;
  classic_stick_right_max_[CWIID_Y] = JOYSTICK_CLASSIC_RIGHT_20PERCENT_MAX_;
  classic_stick_right_min_[CWIID_X] = JOYSTICK_CLASSIC_RIGHT_20PERCENT_MIN_;
  classic_stick_right_min_[CWIID_Y] = JOYSTICK_CLASSIC_RIGHT_20PERCENT_MIN_;
}

void WiimoteNode::publish()
{
  uint8_t joy_subscribers = joy_pub_.getNumSubscribers();
  uint8_t wii_state_subscribers = wiimote_state_pub_.getNumSubscribers();
  uint8_t imu_subscribers = imu_data_pub_.getNumSubscribers();
  uint8_t wii_nunchuk_subscribers = 0;
  uint8_t wii_classic_subscribers = 0;

  uint8_t current_report_mode = wiimote_state_.rpt_mode;
  uint8_t new_report_mode = current_report_mode;

  /*              CWIID_RPT_xxx

                  | | | |M| | |
                  | | | |O| | |
                  | | | |T| | |
                  | | | |I|N|C|
                  | | | |O|U|L|
                  | | | |N|N|A|
                  | | | |P|C|S|
                  | |B|A|L|H|S|
                  |I|T|C|U|U|I|
  ROS_Topic       |R|N|C|S|K|C|
                  |_|_|_|_|_|_|
  joy             | |x|x|x| | |
  imu_data        | | |x|x| | |
  wiimote_state   |x|x|x|x|x| |
  wiimote_nunchuk | | | | |x| |
  wiimote_classic | | | | | |x|
  */

  if (joy_subscribers || wii_state_subscribers || imu_subscribers)
  {
    // Need to collect data on accelerometer and motionplus
    new_report_mode |= (CWIID_RPT_ACC | CWIID_RPT_MOTIONPLUS);

    if (joy_subscribers || wii_state_subscribers)
    {
      // Need to also collect data on buttons
      new_report_mode |= (CWIID_RPT_BTN);
    }

    if (wii_state_subscribers)
    {
      // Need to also collect data on IR and Nunchuk
      new_report_mode |= (CWIID_RPT_IR | CWIID_RPT_NUNCHUK);
    }
  }
  else
  {
    if (!joy_subscribers && !wii_state_subscribers && !imu_subscribers)
    {
      // Can stop collecting data on accelerometer and motionplus
      new_report_mode &= ~(CWIID_RPT_ACC | CWIID_RPT_MOTIONPLUS);
    }

    if (!joy_subscribers && !wii_state_subscribers)
    {
      // Can also stop collecting data on buttons
      new_report_mode &= ~(CWIID_RPT_BTN);
    }

    if (!wii_state_subscribers)
    {
      // Can also stop collecting data on IR
      new_report_mode &= ~(CWIID_RPT_IR);
    }
  }

  // Is the Nunchuk connected?
  if (isPresentNunchuk())
  {
    // Is the Nunchuk publisher not advertised?
    if (nullptr == wiimote_nunchuk_pub_)
    {
      if (!nunchuk_failed_calibration_)
      {
        // The nunchuk was just connected, so read the calibration data
        if (!nunchuk_calibrated_)
        {
          checkFactoryCalibrationData();
        }

        if (nunchuk_calibrated_)
        {
          wiimote_nunchuk_pub_ = nh_.advertise<sensor_msgs::Joy>("/wiimote/nunchuk", 1);
        }
        else
        {
          ROS_ERROR("Topic /wiimote/nunchuk not advertised due to calibration failure");
        }
      }
    }

    wii_nunchuk_subscribers = wiimote_nunchuk_pub_.getNumSubscribers();

    if (wii_nunchuk_subscribers)
    {
      // Need to collect data on nunchuk
      new_report_mode |= (CWIID_RPT_NUNCHUK);
    }
    else
    {
      if (!wii_state_subscribers)
      {
        // Can stop collecting data on nunchuk
        new_report_mode &= ~(CWIID_RPT_NUNCHUK);
      }
    }
  }
  else  // Stop publishing the topic
  {
    // Is the Nunchuk publisher advertised?
    if (nullptr != wiimote_nunchuk_pub_)
    {
      wiimote_nunchuk_pub_.shutdown();

      resetNunchukState();

      if (!wii_state_subscribers)
      {
        // Can stop collecting data on nunchuk
        new_report_mode &= ~(CWIID_RPT_NUNCHUK);
      }
    }

    // If the nunchuk was connect, but failed calibration
    // then attempt to check factory calibration for the wiimote
    if (nunchuk_failed_calibration_)
    {
      checkFactoryCalibrationData();
      nunchuk_failed_calibration_ = false;
    }
  }

  // Is the Classic Pad connected?
  if (isPresentClassic())
  {
    // Is the Classic Pad publisher not advertised?
    if (nullptr == wiimote_classic_pub_)
    {
      wiimote_classic_pub_ = nh_.advertise<sensor_msgs::Joy>("/wiimote/classic", 1);
    }

    wii_classic_subscribers = wiimote_classic_pub_.getNumSubscribers();

    if (wii_classic_subscribers)
    {
      // Need to collect data on classic
      new_report_mode |= (CWIID_RPT_CLASSIC);
    }
    else
    {
      // Can stop collecting data on classic
      new_report_mode &= ~(CWIID_RPT_CLASSIC);
    }
  }
  else  // Stop publishing the topic
  {
    // Is the Classic Pad publisher advertised?
    if (nullptr != wiimote_classic_pub_)
    {
      wiimote_classic_pub_.shutdown();

      resetClassicState();

      // Can stop collecting data on classic
      new_report_mode &= ~(CWIID_RPT_CLASSIC);
    }
  }

  // Update the reporting mode and returning
  if (current_report_mode != new_report_mode)
  {
    setReportMode(new_report_mode);
  }

  if (!joy_subscribers && !wii_state_subscribers && !imu_subscribers &&
      !wii_nunchuk_subscribers & !wii_classic_subscribers)
  {
    // If there are no subscribers, there isn't anything to publish
    return;
  }


  if (!getStateSample())
  {
    // If we can not get State from the Wiimote, there isn't anything to publish
    return;
  }


  if (joy_subscribers)
  {
    // ROS_DEBUG("Number joy subscribers is: %d", joy_subscribers);
    publishJoy();
  }

  if (imu_subscribers)
  {
    // ROS_DEBUG("Number imu_data subscribers is: %d", imu_subscribers);
    publishImuData();
  }

  if (wii_state_subscribers)
  {
    // ROS_DEBUG("Number wiimote_state subscribers is: %d", wii_state_subscribers);
    publishWiimoteState();
  }

  if (wii_nunchuk_subscribers)
  {
    // ROS_DEBUG("Number wiimote_nunchuk subscribers is: %d", wiimote_nunchuk_pub_.getNumSubscribers());
    publishWiimoteNunchuk();
  }

  // Is the Classic Pad connected?
  if (wii_classic_subscribers)
  {
    // ROS_DEBUG("Number wiimote_classic subscribers is: %d", wiimote_classic_pub_.getNumSubscribers());
    publishWiimoteClassic();
  }
}

bool WiimoteNode::getStateSample()
{
  bool result = true;
  bool get_state_result = true;
  bool data_valid = false;

  int count = 0;
  int big_count = 0;
  static int wiimote_count = 0;
  static int motionplus_count = 0;

  do
  {
    get_state_result = (wiimote_c::cwiid_get_state(wiimote_, &wiimote_state_) == 0);

    if (isCollectingWiimote() &&
        (wiimote_state_.acc[CWIID_X] == 0 &&
         wiimote_state_.acc[CWIID_Y] == 0 &&
         wiimote_state_.acc[CWIID_Z] == 0))
    {
      if (count > 1 && !(count % 100))
      {
        // If we can not get valid data from the Wiimote, wait and try again
        ROS_INFO("Waiting for valid wiimote data...");
        count = 0;
        ++big_count;
      }

      usleep(10000);  // wait a hundredth of a second
      ++count;
      if (big_count > 10)
      {
        get_state_result = false;
      }
    }
    else
    {
      if (wiimote_count < IGNORE_DATA_POINTS_)
      {
        // ROS_DEBUG("Ignoring Wiimote data point %d", wiimote_count);
        wiimote_count++;
      }
      else
      {
        data_valid = true;
      }
    }

    usleep(10000);  // wait a hundredth of a second
  }
  while (get_state_result && !data_valid);

  if (isPresentMotionplus())
  {
    data_valid = false;

    count = 0;
    big_count = 0;

    do
    {
      if (wiimote_state_.ext.motionplus.angle_rate[CWIID_PHI] == 0 &&
          wiimote_state_.ext.motionplus.angle_rate[CWIID_THETA] == 0 &&
          wiimote_state_.ext.motionplus.angle_rate[CWIID_PSI] == 0)
      {
        if (count > 1 && !(count % 100))
        {
          // If we can not get valid data from the Wiimote, wait and try again
          ROS_INFO("Waiting for valid MotionPlus data...");
          count = 0;
          ++big_count;
        }

        usleep(10000);  // wait a hundredth of a second
        ++count;
        if (big_count > 10)
        {
          get_state_result = false;
        }
        else
        {
          usleep(10000);  // wait a hundredth of a second
          get_state_result = (wiimote_c::cwiid_get_state(wiimote_, &wiimote_state_) == 0);
        }
      }
      else
      {
        if (motionplus_count < IGNORE_DATA_POINTS_)
        {
          ROS_DEBUG("Ignoring MotionPlus data point %d", motionplus_count);
          motionplus_count++;
          usleep(1000);  // wait a thousandth of a second
        }
        else
        {
          data_valid = true;
          // Get a new data point with valid data
          get_state_result = (wiimote_c::cwiid_get_state(wiimote_, &wiimote_state_) == 0);
        }
      }
    }
    while (get_state_result && !data_valid);
  }
  else
  {
    // MotionPlus was removed, so reset the master count
    motionplus_count = 0;
    resetMotionPlusState();
  }

  if (get_state_result)
  {
    struct timespec state_tv;

    if (clock_gettime(CLOCK_REALTIME, &state_tv) == 0)
    {
      state_secs_ = state_tv.tv_sec;
      state_nsecs_ = state_tv.tv_nsec;
    }
    else
    {
      ROS_ERROR("Error sampling real-time clock");
      result = false;
    }
  }
  else
  {
    result = false;
  }

  return result;
}

void WiimoteNode::setReportMode(uint8_t rpt_mode)
{
  ROS_DEBUG("Change report mode from %d to %d", wiimote_state_.rpt_mode, rpt_mode);

  if (wiimote_c::cwiid_set_rpt_mode(wiimote_, rpt_mode))
  {
    ROS_ERROR("Error setting report mode: Bit(s):%d", rpt_mode);
  }
  else
  {
    wiimote_state_.rpt_mode = rpt_mode;

    // Enable the MotionPlus
    if (rpt_mode & CWIID_RPT_MOTIONPLUS)
    {
      wiimote_c::cwiid_enable(wiimote_, CWIID_FLAG_MOTIONPLUS);
      ROS_DEBUG("Enabled MotionPlus");
    }
  }
}

void WiimoteNode::setLEDBit(uint8_t led, bool on)
{
  uint8_t bit;

  if (led > 3)
  {
    ROS_WARN("LED ID %d out of bounds; ignoring!", led);
  }

  bit = 1 << led;

  if (on)
  {  // Set bit
    led_state_ |= bit;
  }
  else
  {  // Clear bit
    led_state_ &= ~(bit);
  }
}
void WiimoteNode::setRumbleBit(bool on)
{
  if (on)
  {  // Set bit
    rumble_ |= 0x1;
  }
  else
  {  // Clear bit
    rumble_ &= ~(0x1);
  }
}

void WiimoteNode::setLedState(uint8_t led_state)
{
  if (wiimote_c::cwiid_set_led(wiimote_, led_state))
  {
    ROS_ERROR("Error setting LEDs");
  }
}

void WiimoteNode::setRumbleState(uint8_t rumble)
{
  if (wiimote_c::cwiid_set_rumble(wiimote_, rumble))
  {
    ROS_ERROR("Error setting rumble");
  }
}

void WiimoteNode::cwiidErrorCallback(wiimote_c::cwiid_wiimote_t *wiimote, const char *fmt, va_list ap)
{
  const int MAX_BUF = 500;
  char msgs_buf[MAX_BUF];

  vsnprintf(msgs_buf, MAX_BUF, fmt, ap);

  if (wiimote)
  {
    ROS_ERROR("Wii Error: ID: %d: %s", wiimote_c::cwiid_get_id(wiimote), msgs_buf);
  }
  else
  {
    ROS_ERROR("Wii Error: ID: ?: %s", msgs_buf);
  }
}

bool WiimoteNode::isCollectingWiimote()
{
  return wiimote_state_.rpt_mode &
    (CWIID_RPT_BTN | CWIID_RPT_ACC | CWIID_RPT_IR);
}
bool WiimoteNode::isCollectingNunchuk()
{
  return wiimote_state_.rpt_mode & CWIID_RPT_NUNCHUK;
}
bool WiimoteNode::isCollectingClassic()
{
  return wiimote_state_.rpt_mode & CWIID_RPT_CLASSIC;
}
bool WiimoteNode::isCollectingMotionplus()
{
  return wiimote_state_.rpt_mode & CWIID_RPT_MOTIONPLUS;
}

bool WiimoteNode::isPresentNunchuk()
{
  return wiimote_state_.ext_type == wiimote_c::CWIID_EXT_NUNCHUK;
}
bool WiimoteNode::isPresentClassic()
{
  return wiimote_state_.ext_type == wiimote_c::CWIID_EXT_CLASSIC;
}
bool WiimoteNode::isPresentMotionplus()
{
  return wiimote_state_.ext_type == wiimote_c::CWIID_EXT_MOTIONPLUS;
}

bool WiimoteNode::calibrateJoystick(uint8_t stick[2], uint8_t (&center)[2], const char *name)
{
  bool is_calibrated = false;

  // Grab the current Joystick position as center
  // If it is not reporting 0, 0
  if (stick[CWIID_X] != 0 && stick[CWIID_Y] != 0)
  {
    center[CWIID_X] = stick[CWIID_X];
    center[CWIID_Y] = stick[CWIID_Y];

    is_calibrated = true;

    ROS_DEBUG("%s Joystick Center:: x:%d, y:%d", name, center[CWIID_X], center[CWIID_Y]);
  }

  return is_calibrated;
}

void WiimoteNode::updateJoystickMinMax(uint8_t stick[2], uint8_t (&stick_min)[2],
    uint8_t (&stick_max)[2], const char *name)
{
  bool updated = false;

  if (stick[CWIID_X] < stick_min[CWIID_X])
  {
    stick_min[CWIID_X] = stick[CWIID_X];
    updated = true;
  }

  if (stick[CWIID_Y] < stick_min[CWIID_Y])
  {
    stick_min[CWIID_Y] = stick[CWIID_Y];
    updated = true;
  }

  if (stick[CWIID_X] > stick_max[CWIID_X])
  {
    stick_max[CWIID_X] = stick[CWIID_X];
    updated = true;
  }

  if (stick[CWIID_Y] > stick_max[CWIID_Y])
  {
    stick_max[CWIID_Y] = stick[CWIID_Y];
    updated = true;
  }

  if (updated)
  {
    ROS_DEBUG("%s Joystick:: Min x:%3d, y:%3d  Max x:%3d, y:%3d", name,
        stick_min[CWIID_X], stick_min[CWIID_Y], stick_max[CWIID_X], stick_max[CWIID_Y]);
  }
}

void WiimoteNode::calculateJoystickAxisXY(uint8_t stick_current[2], uint8_t stick_min[2],
    uint8_t stick_max[2], uint8_t stick_center[2], double (&stick)[2])
{
  double deadzone[2];
  int deadzoneMargin = 4;

  // Scale the Wiimote Joystick integer values (0-31, 63, or 255)
  // to a double value between 0.0 and 1.0.

  // The width of the center deadzone needs to scale
  // with the resolution of the joystick in use.
  // Nunchuk range is 0-255; defaults to 4
  // Classic Left range is 0-63; set to 2
  // Classic Right range is 0-31; set to 1
  // Original Python implementation was always 0.05 for all

  // Optimize for Nunchuk case; most common
  if (stick_max[CWIID_X] < 128)
  {
    if (stick_max[CWIID_X] < 32)
    {
      deadzoneMargin = 1;
    }
    else if (stick_max[CWIID_X] < 64)
    {
      deadzoneMargin = 2;
    }
    else
    {
      // No known wiimote joystick uses this range
      deadzoneMargin = 3;
    }
  }

  if (stick_current[CWIID_X] > stick_center[CWIID_X])
  {
    stick[CWIID_X] = -(stick_current[CWIID_X] - stick_center[CWIID_X]) /
      ((stick_max[CWIID_X] - stick_center[CWIID_X]) * 1.0);
    deadzone[CWIID_X] = deadzoneMargin /
      ((stick_max[CWIID_X] - stick_center[CWIID_X]) * 1.0);
  }
  else
  {
    stick[CWIID_X] = -(stick_current[CWIID_X] - stick_center[CWIID_X]) /
      ((stick_center[CWIID_X] - stick_min[CWIID_X]) * 1.0);
    deadzone[CWIID_X] = deadzoneMargin /
      ((stick_center[CWIID_X] - stick_min[CWIID_X]) * 1.0);
  }
  if (stick_current[CWIID_Y] > stick_center[CWIID_Y])
  {
    stick[CWIID_Y] = (stick_current[CWIID_Y] - stick_center[CWIID_Y]) /
      ((stick_max[CWIID_Y] - stick_center[CWIID_Y]) * 1.0);
    deadzone[CWIID_Y] = deadzoneMargin /
      ((stick_max[CWIID_Y] - stick_center[CWIID_Y]) * 1.0);
  }
  else
  {
    stick[CWIID_Y] = (stick_current[CWIID_Y] - stick_center[CWIID_Y]) /
      ((stick_center[CWIID_Y] - stick_min[CWIID_Y]) * 1.0);
    deadzone[CWIID_Y] = deadzoneMargin /
      ((stick_center[CWIID_Y] - stick_min[CWIID_Y]) * 1.0);
  }

  // Create a deadzone in the center
  if (fabs(stick[CWIID_X]) <= deadzone[CWIID_X])
  {
    stick[CWIID_X] = 0.0;
  }
  if (fabs(stick[CWIID_Y]) <= deadzone[CWIID_Y])
  {
    stick[CWIID_Y] = 0.0;
  }
}

void WiimoteNode::publishJoy()
{
  sensor_msgs::Joy joy_data;

  joy_data.header.stamp.sec = state_secs_;
  joy_data.header.stamp.nsec = state_nsecs_;

  joy_data.axes.push_back(zeroedByCal(wiimote_state_.acc[CWIID_X],
        wiimote_cal_.zero[CWIID_X], wiimote_cal_.one[CWIID_X]) * EARTH_GRAVITY_);
  joy_data.axes.push_back(zeroedByCal(wiimote_state_.acc[CWIID_Y],
        wiimote_cal_.zero[CWIID_Y], wiimote_cal_.one[CWIID_Y]) * EARTH_GRAVITY_);
  joy_data.axes.push_back(zeroedByCal(wiimote_state_.acc[CWIID_Z],
        wiimote_cal_.zero[CWIID_Z], wiimote_cal_.one[CWIID_Z]) * EARTH_GRAVITY_);

  // NOTE: Order is different for /wiimote/state
  // Keep consistency with existing Python Node
  joy_data.buttons.push_back((wiimote_state_.buttons & CWIID_BTN_1) > 0);
  joy_data.buttons.push_back((wiimote_state_.buttons & CWIID_BTN_2) > 0);
  joy_data.buttons.push_back((wiimote_state_.buttons & CWIID_BTN_A) > 0);
  joy_data.buttons.push_back((wiimote_state_.buttons & CWIID_BTN_B) > 0);
  joy_data.buttons.push_back((wiimote_state_.buttons & CWIID_BTN_PLUS) > 0);
  joy_data.buttons.push_back((wiimote_state_.buttons & CWIID_BTN_MINUS) > 0);
  joy_data.buttons.push_back((wiimote_state_.buttons & CWIID_BTN_LEFT) > 0);
  joy_data.buttons.push_back((wiimote_state_.buttons & CWIID_BTN_RIGHT) > 0);
  joy_data.buttons.push_back((wiimote_state_.buttons & CWIID_BTN_UP) > 0);
  joy_data.buttons.push_back((wiimote_state_.buttons & CWIID_BTN_DOWN) > 0);
  joy_data.buttons.push_back((wiimote_state_.buttons & CWIID_BTN_HOME) > 0);

  joy_pub_.publish(joy_data);
}

void WiimoteNode::publishImuData()
{
  // The Wiimote provides the Acceleration and optionally Gyro
  // if MotionPlus is available, but not orientation information.

  sensor_msgs::Imu imu_data_data;

  imu_data_data.header.stamp.sec = state_secs_;
  imu_data_data.header.stamp.nsec = state_nsecs_;

  // Publish that Orientation data is invalid
  // [ -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
  imu_data_data.orientation_covariance[0] = -1.0;

  // Acceleration
  imu_data_data.linear_acceleration.x = zeroedByCal(wiimote_state_.acc[CWIID_X],
      wiimote_cal_.zero[CWIID_X], wiimote_cal_.one[CWIID_X]) * EARTH_GRAVITY_;
  imu_data_data.linear_acceleration.y = zeroedByCal(wiimote_state_.acc[CWIID_Y],
      wiimote_cal_.zero[CWIID_Y], wiimote_cal_.one[CWIID_Y]) * EARTH_GRAVITY_;
  imu_data_data.linear_acceleration.z = zeroedByCal(wiimote_state_.acc[CWIID_Z],
      wiimote_cal_.zero[CWIID_Z], wiimote_cal_.one[CWIID_Z]) * EARTH_GRAVITY_;

  imu_data_data.linear_acceleration_covariance = linear_acceleration_covariance_;

  // MotionPlus Gyro
  if (isPresentMotionplus())
  {
    imu_data_data.angular_velocity.x = (wiimote_state_.ext.motionplus.angle_rate[CWIID_PHI] -
        angular_velocity_stat_.getMeanRaw()[CWIID_PHI]) * GYRO_SCALE_FACTOR_;
    imu_data_data.angular_velocity.y = (wiimote_state_.ext.motionplus.angle_rate[CWIID_THETA] -
        angular_velocity_stat_.getMeanRaw()[CWIID_THETA]) * GYRO_SCALE_FACTOR_;
    imu_data_data.angular_velocity.z = (wiimote_state_.ext.motionplus.angle_rate[CWIID_PSI] -
        angular_velocity_stat_.getMeanRaw()[CWIID_PSI]) * GYRO_SCALE_FACTOR_;
  }

  imu_data_data.angular_velocity_covariance = angular_velocity_covariance_;

  imu_data_pub_.publish(imu_data_data);
}

void WiimoteNode::publishWiimoteState()
{
  wiimote::State wiimote_state_data;

  wiimote_state_data.header.stamp.sec = state_secs_;
  wiimote_state_data.header.stamp.nsec = state_nsecs_;

  // Wiimote data
  wiimote_state_data.linear_acceleration_zeroed.x = zeroedByCal(wiimote_state_.acc[CWIID_X],
      wiimote_cal_.zero[CWIID_X], wiimote_cal_.one[CWIID_X]) * EARTH_GRAVITY_;
  wiimote_state_data.linear_acceleration_zeroed.y = zeroedByCal(wiimote_state_.acc[CWIID_Y],
      wiimote_cal_.zero[CWIID_Y], wiimote_cal_.one[CWIID_Y]) * EARTH_GRAVITY_;
  wiimote_state_data.linear_acceleration_zeroed.z = zeroedByCal(wiimote_state_.acc[CWIID_Z],
      wiimote_cal_.zero[CWIID_Z], wiimote_cal_.one[CWIID_Z]) * EARTH_GRAVITY_;

  wiimote_state_data.linear_acceleration_raw.x = wiimote_state_.acc[CWIID_X];
  wiimote_state_data.linear_acceleration_raw.y = wiimote_state_.acc[CWIID_Y];
  wiimote_state_data.linear_acceleration_raw.z = wiimote_state_.acc[CWIID_Z];

  wiimote_state_data.linear_acceleration_covariance = linear_acceleration_covariance_;

  // MotionPlus Gyro
  wiimote_state_data.angular_velocity_covariance = angular_velocity_covariance_;

  if (isPresentMotionplus())
  {
    wiimote_state_data.angular_velocity_zeroed.x = (wiimote_state_.ext.motionplus.angle_rate[CWIID_PHI] -
        angular_velocity_stat_.getMeanRaw()[CWIID_PHI]) * GYRO_SCALE_FACTOR_;
    wiimote_state_data.angular_velocity_zeroed.y = (wiimote_state_.ext.motionplus.angle_rate[CWIID_THETA] -
        angular_velocity_stat_.getMeanRaw()[CWIID_THETA]) * GYRO_SCALE_FACTOR_;
    wiimote_state_data.angular_velocity_zeroed.z = (wiimote_state_.ext.motionplus.angle_rate[CWIID_PSI] -
        angular_velocity_stat_.getMeanRaw()[CWIID_PSI]) * GYRO_SCALE_FACTOR_;

    wiimote_state_data.angular_velocity_raw.x = wiimote_state_.ext.motionplus.angle_rate[CWIID_PHI];
    wiimote_state_data.angular_velocity_raw.y = wiimote_state_.ext.motionplus.angle_rate[CWIID_THETA];
    wiimote_state_data.angular_velocity_raw.z = wiimote_state_.ext.motionplus.angle_rate[CWIID_PSI];
  }

  // NOTE: Order is different for /joy
  // Keep consistency with existing Python Node
  wiimote_state_data.buttons.elems[ 0] = ((wiimote_state_.buttons & CWIID_BTN_1) > 0);
  wiimote_state_data.buttons.elems[ 1] = ((wiimote_state_.buttons & CWIID_BTN_2) > 0);
  wiimote_state_data.buttons.elems[ 2] = ((wiimote_state_.buttons & CWIID_BTN_PLUS) > 0);
  wiimote_state_data.buttons.elems[ 3] = ((wiimote_state_.buttons & CWIID_BTN_MINUS) > 0);
  wiimote_state_data.buttons.elems[ 4] = ((wiimote_state_.buttons & CWIID_BTN_A) > 0);
  wiimote_state_data.buttons.elems[ 5] = ((wiimote_state_.buttons & CWIID_BTN_B) > 0);
  wiimote_state_data.buttons.elems[ 6] = ((wiimote_state_.buttons & CWIID_BTN_UP) > 0);
  wiimote_state_data.buttons.elems[ 7] = ((wiimote_state_.buttons & CWIID_BTN_DOWN) > 0);
  wiimote_state_data.buttons.elems[ 8] = ((wiimote_state_.buttons & CWIID_BTN_LEFT) > 0);
  wiimote_state_data.buttons.elems[ 9] = ((wiimote_state_.buttons & CWIID_BTN_RIGHT) > 0);
  wiimote_state_data.buttons.elems[10] = ((wiimote_state_.buttons & CWIID_BTN_HOME) > 0);

  // Nunchuk data
  if (isPresentNunchuk())
  {
    if (publishWiimoteNunchukCommon())
    {
      // Joy stick
      double stick[2];

      calculateJoystickAxisXY(wiimote_state_.ext.nunchuk.stick, nunchuk_stick_min_,
          nunchuk_stick_max_, nunchuk_stick_center_, stick);

      wiimote_state_data.nunchuk_joystick_raw[CWIID_X] = wiimote_state_.ext.nunchuk.stick[CWIID_X];
      wiimote_state_data.nunchuk_joystick_raw[CWIID_Y] = wiimote_state_.ext.nunchuk.stick[CWIID_Y];

      wiimote_state_data.nunchuk_joystick_zeroed[CWIID_X] = stick[CWIID_X];
      wiimote_state_data.nunchuk_joystick_zeroed[CWIID_Y] = stick[CWIID_Y];

      wiimote_state_data.nunchuk_acceleration_raw.x = wiimote_state_.ext.nunchuk.acc[CWIID_X];
      wiimote_state_data.nunchuk_acceleration_raw.y = wiimote_state_.ext.nunchuk.acc[CWIID_Y];
      wiimote_state_data.nunchuk_acceleration_raw.z = wiimote_state_.ext.nunchuk.acc[CWIID_Z];

      wiimote_state_data.nunchuk_acceleration_zeroed.x =
        zeroedByCal(wiimote_state_.ext.nunchuk.acc[CWIID_X],
            nunchuk_cal_.zero[CWIID_X], nunchuk_cal_.one[CWIID_X]) * EARTH_GRAVITY_;
      wiimote_state_data.nunchuk_acceleration_zeroed.y =
        zeroedByCal(wiimote_state_.ext.nunchuk.acc[CWIID_Y],
            nunchuk_cal_.zero[CWIID_Y], nunchuk_cal_.one[CWIID_Y]) * EARTH_GRAVITY_;
      wiimote_state_data.nunchuk_acceleration_zeroed.z =
        zeroedByCal(wiimote_state_.ext.nunchuk.acc[CWIID_Z],
            nunchuk_cal_.zero[CWIID_Z], nunchuk_cal_.one[CWIID_Z]) * EARTH_GRAVITY_;

      // Keep consistency with existing Python Node
      wiimote_state_data.nunchuk_buttons[0] =
        ((wiimote_state_.ext.nunchuk.buttons & CWIID_NUNCHUK_BTN_Z) > 0);
      wiimote_state_data.nunchuk_buttons[1] =
        ((wiimote_state_.ext.nunchuk.buttons & CWIID_NUNCHUK_BTN_C) > 0);
    }
  }

  // IR Tracking Data
  for (int ir_idx = 0; ir_idx < CWIID_IR_SRC_COUNT; ir_idx++)
  {
    wiimote::IrSourceInfo irSourceInfo;

    if (wiimote_state_.ir_src[ir_idx].valid)
    {
      irSourceInfo.x = wiimote_state_.ir_src[ir_idx].pos[CWIID_X];
      irSourceInfo.y = wiimote_state_.ir_src[ir_idx].pos[CWIID_Y];

      irSourceInfo.ir_size = wiimote_state_.ir_src[ir_idx].size;

      if (irSourceInfo.ir_size < 1)
      {
        irSourceInfo.ir_size = wiimote::State::INVALID;
      }
    }
    else
    {
      irSourceInfo.x = wiimote::State::INVALID_FLOAT;
      irSourceInfo.y = wiimote::State::INVALID_FLOAT;

      irSourceInfo.ir_size = wiimote::State::INVALID;
    }

    wiimote_state_data.ir_tracking.push_back(irSourceInfo);
  }

  // LEDs / Rumble
  for (uint8_t i = 0; i < 4; i++)
  {
    wiimote_state_data.LEDs[i] = wiimote_state_.led & (0x1 << i);
  }
  wiimote_state_data.rumble = wiimote_state_.rumble & 0x1;

  // Battery
  wiimote_state_data.raw_battery = wiimote_state_.battery;
  wiimote_state_data.percent_battery = wiimote_state_.battery * 100.0 / CWIID_BATTERY_MAX;

  // Zeroing time (aka Calibration time)
  wiimote_state_data.zeroing_time = calibration_time_;

  // Wiimote state errors
  // No usage found in original Python code which every set this variable
  // TODO(mdhorn): Use this to report error
  // Is this a count? or a bunch of status that are ORed together?
  wiimote_state_data.errors = wiimote_errors;

  wiimote_state_pub_.publish(wiimote_state_data);
}

bool WiimoteNode::publishWiimoteNunchukCommon()
{
  // Testing different Nunchuks show that they have different
  // centers and different range of motion.
  //
  // Best Approximation:
  // Grabbing the first set of x, y with the assumption of
  // the joystick is centered.
  // We know the joystick can only report between 0 and 255
  // for each axis; so assume a 20% guard band to set the
  // initial minimums and maximums. As the joystick is used,
  // updated the min/max values based on observed data.
  // This will have the effect of a less throw of the joystick
  // throttle during first movement to the max/min position.
  //
  // TODO(mdhorn): Could be improved by a true user interaction calibration
  // to find the center of the joystick, the max and min for each x and y.
  // But the effort in coding and extra burden on the one the user
  // may not be warranted.

  bool result = true;

  if (isPresentNunchuk())
  {
    if (!nunchuk_stick_calibrated_)
    {
      nunchuk_stick_calibrated_ = calibrateJoystick(
          wiimote_state_.ext.nunchuk.stick, nunchuk_stick_center_, "Nunchuk");

      // Don't publish if we haven't found the center position
      if (!nunchuk_stick_calibrated_)
      {
        result = false;
      }
    }

    updateJoystickMinMax(wiimote_state_.ext.nunchuk.stick, nunchuk_stick_min_,
        nunchuk_stick_max_, "Nunchuk");
  }
  else
  {
    ROS_WARN("State type is not Nunchuk!");
    result = false;
  }

  return result;
}

void WiimoteNode::publishWiimoteNunchuk()
{
  sensor_msgs::Joy wiimote_nunchuk_data;

  if (publishWiimoteNunchukCommon())
  {
    wiimote_nunchuk_data.header.stamp.sec = state_secs_;
    wiimote_nunchuk_data.header.stamp.nsec = state_nsecs_;

    // Joy stick
    double stick[2];

    calculateJoystickAxisXY(wiimote_state_.ext.nunchuk.stick, nunchuk_stick_min_,
        nunchuk_stick_max_, nunchuk_stick_center_, stick);

    wiimote_nunchuk_data.axes.push_back(stick[CWIID_X]);  // x
    wiimote_nunchuk_data.axes.push_back(stick[CWIID_Y]);  // y

    wiimote_nunchuk_data.axes.push_back(zeroedByCal(wiimote_state_.ext.nunchuk.acc[CWIID_X],
          nunchuk_cal_.zero[CWIID_X], nunchuk_cal_.one[CWIID_X]) * EARTH_GRAVITY_);
    wiimote_nunchuk_data.axes.push_back(zeroedByCal(wiimote_state_.ext.nunchuk.acc[CWIID_Y],
          nunchuk_cal_.zero[CWIID_Y], nunchuk_cal_.one[CWIID_Y]) * EARTH_GRAVITY_);
    wiimote_nunchuk_data.axes.push_back(zeroedByCal(wiimote_state_.ext.nunchuk.acc[CWIID_Z],
          nunchuk_cal_.zero[CWIID_Z], nunchuk_cal_.one[CWIID_Z]) * EARTH_GRAVITY_);


    // NOTE: Order is different for /wiimote/state
    // Keep consistency with existing Python Node
    wiimote_nunchuk_data.buttons.push_back((wiimote_state_.ext.nunchuk.buttons & CWIID_NUNCHUK_BTN_Z) > 0);
    wiimote_nunchuk_data.buttons.push_back((wiimote_state_.ext.nunchuk.buttons & CWIID_NUNCHUK_BTN_C) > 0);

    wiimote_nunchuk_pub_.publish(wiimote_nunchuk_data);
  }
}

void WiimoteNode::publishWiimoteClassic()
{
  // Using the same "Best Approximation" methods from
  // WiimoteNode::publishWiimoteNunchuk()

  sensor_msgs::Joy wiimote_classic_data;

  if (isPresentClassic())
  {
    ROS_WARN("State type is not Classic!");
    return;
  }

  if (!classic_stick_left_calibrated_)
  {
    classic_stick_left_calibrated_ = calibrateJoystick(
        wiimote_state_.ext.classic.l_stick, classic_stick_left_center_, "Classic Left");
  }

  if (!classic_stick_right_calibrated_)
  {
    classic_stick_right_calibrated_ = calibrateJoystick(
        wiimote_state_.ext.classic.r_stick, classic_stick_right_center_, "Classic Right");
  }

  if ((!classic_stick_left_calibrated_) ||
      (!classic_stick_right_calibrated_))
  {
    // Don't publish if we haven't found the center positions
    return;
  }

  updateJoystickMinMax(wiimote_state_.ext.classic.l_stick, classic_stick_left_min_,
      classic_stick_left_max_, "Classic Left");
  updateJoystickMinMax(wiimote_state_.ext.classic.r_stick, classic_stick_right_min_,
      classic_stick_right_max_, "Classic Right");

  wiimote_classic_data.header.stamp.sec = state_secs_;
  wiimote_classic_data.header.stamp.nsec = state_nsecs_;

  // Joy stick
  double stick_left[2];
  double stick_right[2];

  calculateJoystickAxisXY(wiimote_state_.ext.classic.l_stick, classic_stick_left_min_,
      classic_stick_left_max_, classic_stick_left_center_, stick_left);
  calculateJoystickAxisXY(wiimote_state_.ext.classic.r_stick, classic_stick_right_min_,
      classic_stick_right_max_, classic_stick_right_center_, stick_right);

  wiimote_classic_data.axes.push_back(stick_left[CWIID_X]);   // Left x
  wiimote_classic_data.axes.push_back(stick_left[CWIID_Y]);   // Left y
  wiimote_classic_data.axes.push_back(stick_right[CWIID_X]);  // Right x
  wiimote_classic_data.axes.push_back(stick_right[CWIID_Y]);  // Right y

  // NOTE: Order is different for /wiimote/state
  // Keep consistency with existing Python Node
  wiimote_classic_data.buttons.push_back((wiimote_state_.ext.classic.buttons & CWIID_CLASSIC_BTN_X) > 0);
  wiimote_classic_data.buttons.push_back((wiimote_state_.ext.classic.buttons & CWIID_CLASSIC_BTN_Y) > 0);
  wiimote_classic_data.buttons.push_back((wiimote_state_.ext.classic.buttons & CWIID_CLASSIC_BTN_A) > 0);
  wiimote_classic_data.buttons.push_back((wiimote_state_.ext.classic.buttons & CWIID_CLASSIC_BTN_B) > 0);
  wiimote_classic_data.buttons.push_back((wiimote_state_.ext.classic.buttons & CWIID_CLASSIC_BTN_PLUS) > 0);
  wiimote_classic_data.buttons.push_back((wiimote_state_.ext.classic.buttons & CWIID_CLASSIC_BTN_MINUS) > 0);
  wiimote_classic_data.buttons.push_back((wiimote_state_.ext.classic.buttons & CWIID_CLASSIC_BTN_LEFT) > 0);
  wiimote_classic_data.buttons.push_back((wiimote_state_.ext.classic.buttons & CWIID_CLASSIC_BTN_RIGHT) > 0);
  wiimote_classic_data.buttons.push_back((wiimote_state_.ext.classic.buttons & CWIID_CLASSIC_BTN_UP) > 0);
  wiimote_classic_data.buttons.push_back((wiimote_state_.ext.classic.buttons & CWIID_CLASSIC_BTN_DOWN) > 0);
  wiimote_classic_data.buttons.push_back((wiimote_state_.ext.classic.buttons & CWIID_CLASSIC_BTN_HOME) > 0);
  wiimote_classic_data.buttons.push_back((wiimote_state_.ext.classic.buttons & CWIID_CLASSIC_BTN_L) > 0);
  wiimote_classic_data.buttons.push_back((wiimote_state_.ext.classic.buttons & CWIID_CLASSIC_BTN_R) > 0);
  wiimote_classic_data.buttons.push_back((wiimote_state_.ext.classic.buttons & CWIID_CLASSIC_BTN_ZL) > 0);
  wiimote_classic_data.buttons.push_back((wiimote_state_.ext.classic.buttons & CWIID_CLASSIC_BTN_ZR) > 0);

  wiimote_classic_pub_.publish(wiimote_classic_data);
}

// const sensor_msgs::JoyFeedbackArray::ConstPtr& feedback is unused
void WiimoteNode::joySetFeedbackCallback(const sensor_msgs::JoyFeedbackArray::ConstPtr& feedback)
{
  bool led_found = false;
  bool rumble_found = false;

  for (std::vector<sensor_msgs::JoyFeedback>::const_iterator it = feedback->array.begin();
      it != feedback->array.end(); ++it)
  {
    if ((*it).type == sensor_msgs::JoyFeedback::TYPE_LED)
    {
      led_found = true;

      if ((*it).intensity >= 0.5)
      {
        setLEDBit((*it).id, true);
      }
      else
      {
        setLEDBit((*it).id, false);
      }
    }
    else if ((*it).type == sensor_msgs::JoyFeedback::TYPE_RUMBLE)
    {
      if ((*it).id > 0)
      {
        ROS_WARN("RUMBLE ID %d out of bounds; ignoring!", (*it).id);
      }
      else
      {
        rumble_found = true;

        if ((*it).intensity >= 0.5)
        {
          setRumbleBit(true);
        }
        else
        {
          setRumbleBit(false);
        }
      }
    }
    else
    {
      ROS_WARN("Unknown JoyFeedback command; ignored");
    }
  }

  if (led_found)
  {
    setLedState(led_state_);
  }

  if (rumble_found)
  {
    setRumbleState(rumble_);
  }
}

bool WiimoteNode::serviceImuCalibrateCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  // Publish the new calibration state
  checkFactoryCalibrationData();

  return true;
}

WiimoteNode *g_wiimote_node;

// int sig The signal number is unused
void mySigHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.

  // Clear the lights and rumble
  if (nullptr != g_wiimote_node)
  {
    g_wiimote_node->setRumbleState(0);
    g_wiimote_node->setLedState(0);
  }

  // All the default sigint handler does is call shutdown()
  ros::shutdown();

  exit(0);
}

int main(int argc, char *argv[])
{
  bool fed_addr = false;
  std::string bluetooth_addr;
  ros::init(argc, argv, "wiimote_controller");

  g_wiimote_node = new WiimoteNode();
  // Do we have a bluetooth address passed in?
  if (argc > 1)
  {
    ROS_INFO("Using Bluetooth address specified from CLI");
    g_wiimote_node->setBluetoothAddr(argv[1]);
    fed_addr = true;
  }

  if (ros::param::get("~bluetooth_addr", bluetooth_addr))
  {
    g_wiimote_node->setBluetoothAddr(bluetooth_addr.c_str());
    fed_addr = true;
  }

  int pair_timeout;
  ros::param::param<int>("~pair_timeout", pair_timeout, 5);

  if (fed_addr)
    ROS_INFO("* * * Pairing with %s", g_wiimote_node->getBluetoothAddr());

  else
    ROS_INFO("Searching for Wiimotes");

  ROS_INFO("Allow all joy sticks to remain at center position until calibrated.");

  if (g_wiimote_node->pairWiimote(0, pair_timeout))
  {
    ROS_INFO("Wiimote is Paired");

    signal(SIGINT, mySigHandler);
    signal(SIGTERM, mySigHandler);
  }
  else
  {
    ROS_ERROR("* * * Wiimote pairing failed.");
    ros::shutdown();
  }

  ros::Rate loop_rate(10);  // 10Hz

  while (ros::ok())
  {
    g_wiimote_node->publish();

    ros::spinOnce();

    loop_rate.sleep();
  }

  if (g_wiimote_node->unpairWiimote())
  {
    ROS_ERROR("Error on wiimote disconnect");
    return -1;
  }

  return 0;
}
