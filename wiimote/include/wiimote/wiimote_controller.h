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
 * Revisions:
 *
 */

#pragma once
#ifndef WIIMOTE_WIIMOTE_CONTROLLER_H
#define WIIMOTE_WIIMOTE_CONTROLLER_H

#include "ros/ros.h"
#include "sensor_msgs/JoyFeedbackArray.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Imu.h"

#include "wiimote/stat_vector_3d.h"

// We need to link against these
#include <bluetooth/bluetooth.h>  // libbluetooth.so
namespace wiimote_c
{
#include <cwiid.h>  // libcwiid.so
}

#define zeroedByCal(raw, zero, one) \
  (((raw - zero) * 1.0) / ((one - zero) * 1.0))

class WiimoteNode
{
public:
  WiimoteNode();
  ~WiimoteNode();
  char *getBluetoothAddr();
  void setBluetoothAddr(const char *bt_str);
  bool pairWiimote(int flags, int timeout);
  int unpairWiimote();

  void publish();

  void setLedState(uint8_t led_state);
  void setRumbleState(uint8_t rumble);

private:
  void setReportMode(uint8_t rpt_mode);
  void checkFactoryCalibrationData();
  void resetMotionPlusState();
  void resetNunchukState();
  void resetClassicState();

  ros::NodeHandle nh_;

  static wiimote_c::cwiid_err_t cwiidErrorCallback;

/**
  Node [/wiimote_controller]
  Publications:
   * /joy [sensor_msgs/Joy]
   * /wiimote/state [wiimote/State]
   * /wiimote/nunchuk [sensor_msgs/Joy]
   * /wiimote/classic [sensor_msgs/Joy]
   * /imu/is_calibrated [std_msgs/Bool]
   * /imu/data [sensor_msgs/Imu]

  Subscriptions:
   * /joy/set_feedback [sensor_msgs/JoyFeedbackArray]

  Services:
   * /imu/calibrate
**/
  void joySetFeedbackCallback(const sensor_msgs::JoyFeedbackArray::ConstPtr& feedback);
  bool serviceImuCalibrateCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  bool isCollectingWiimote();
  bool isCollectingNunchuk();
  bool isCollectingClassic();
  bool isCollectingMotionplus();

  bool isPresentNunchuk();
  bool isPresentClassic();
  bool isPresentMotionplus();

  bool calibrateJoystick(uint8_t stick[2], uint8_t (&center)[2], const char *name);
  void updateJoystickMinMax(uint8_t stick[2], uint8_t (&stick_min)[2],
      uint8_t (&stick_max)[2], const char *name);
  void calculateJoystickAxisXY(uint8_t stick_current[2], uint8_t stick_min[2],
      uint8_t stick_max[2], uint8_t stick_center[2], double (&stick)[2]);

  void publishJoy();
  void publishImuData();
  void publishWiimoteState();
  bool publishWiimoteNunchukCommon();
  void publishWiimoteNunchuk();
  void publishWiimoteClassic();

  bool getStateSample();

  void setLEDBit(uint8_t led, bool on);
  void setRumbleBit(bool on);

  ros::Publisher joy_pub_;
  ros::Publisher imu_data_pub_;
  ros::Publisher wiimote_state_pub_;
  ros::Publisher wiimote_nunchuk_pub_;
  ros::Publisher wiimote_classic_pub_;
  ros::Publisher imu_is_calibrated_pub_;

  ros::Subscriber joy_set_feedback_sub_;

  ros::ServiceServer imu_calibrate_srv_;

  // bluetooth device address
  bdaddr_t bt_device_addr_;

  // wiimote handle
  wiimote_c::cwiid_wiimote_t *wiimote_;

  // Last state of the Wiimote
  struct wiimote_c::cwiid_state wiimote_state_;
  void initializeWiimoteState();
  // Time last state sample was taken
  uint32_t state_secs_;
  uint32_t state_nsecs_;

  // Which data items should be reported in state
  uint8_t report_mode_;

  // Calibration status Wiimote
  struct wiimote_c::acc_cal wiimote_cal_;  // wiimote acceleration factory calibration
  bool wiimote_calibrated_;
  ros::Time calibration_time_;

  // Joystick related constants
  const uint8_t JOYSTICK_NUNCHUK_DEFAULT_CENTER_ = 127;       // Theoretical center
  const uint8_t JOYSTICK_NUNCHUK_20PERCENT_MAX_ = 205;
  const uint8_t JOYSTICK_NUNCHUK_20PERCENT_MIN_ = 50;
  const uint8_t JOYSTICK_CLASSIC_LEFT_DEFAULT_CENTER_ = 31;   // Theoretical center
  const uint8_t JOYSTICK_CLASSIC_LEFT_20PERCENT_MAX_ = 50;
  const uint8_t JOYSTICK_CLASSIC_LEFT_20PERCENT_MIN_ = 13;
  const uint8_t JOYSTICK_CLASSIC_RIGHT_DEFAULT_CENTER_ = 15;  // Theoretical center
  const uint8_t JOYSTICK_CLASSIC_RIGHT_20PERCENT_MAX_ = 25;
  const uint8_t JOYSTICK_CLASSIC_RIGHT_20PERCENT_MIN_ = 6;

  // Calibration status Nunchuk
  struct wiimote_c::acc_cal nunchuk_cal_;  // nunchuk acceleration factory calibration
  bool nunchuk_calibrated_;
  bool nunchuk_failed_calibration_;
  uint8_t nunchuk_stick_center_[2];  // nunchuk stick center position
  bool nunchuk_stick_calibrated_;
  uint8_t nunchuk_stick_max_[2];  // Maximums x,y
  uint8_t nunchuk_stick_min_[2];  // Minimums x,y

  // Calibration status Classic Controller
  uint8_t classic_stick_left_center_[2];   // nunchuk stick center position
  bool classic_stick_left_calibrated_;
  uint8_t classic_stick_left_max_[2];
  uint8_t classic_stick_left_min_[2];
  uint8_t classic_stick_right_center_[2];  // nunchuk stick center position
  bool classic_stick_right_calibrated_;
  uint8_t classic_stick_right_max_[2];
  uint8_t classic_stick_right_min_[2];

  const int IGNORE_DATA_POINTS_ = 100;  // throw away the first few data points
  const int COVARIANCE_DATA_POINTS_ = 100;
  StatVector3d linear_acceleration_stat_;
  StatVector3d angular_velocity_stat_;
  boost::array<double, 9> linear_acceleration_covariance_;
  boost::array<double, 9> angular_velocity_covariance_;

  uint8_t led_state_ = 0;
  uint8_t rumble_ = 0;

  uint64_t wiimote_errors = 0;

  // Convert wiimote accelerator readings from g's to m/sec^2:
  const double EARTH_GRAVITY_ = 9.80665;  // m/sec^2 @sea_level

  // Method used in Wiimote Python version
  // TODO(mdhorn): Repeat experiment or create a new one
  // and collect data from multiple wiimotes and use mean.
  //
  // Turning wiimote gyro readings to radians/sec.
  // This scale factor is highly approximate. Procedure:
  //    - Tape Wiimote to center of an office chair seat
  //    - Rotate the chair at approximately constant speed
  //      for 10 seconds. This resulted in 6 chair revolutions
  //    - On average, the Wiimote gyro read 3570 during this
  //      experiment.
  //    - Speed of chair revolving:
  //         * One full circle is: 2#pi radians
  //         * Six revolutions = 12pi radians. ==> 12pi rad in 10 sec ==> 1.2pi rad/sec
  //         * => 3570 == 1.2pi
  //         * => x*3570 = 1.2pi
  //         * => x = 1.2pi/3570 (1.2pi = 3.769908)
  //         * => scale factor = 0.001055997
  // So multiplying the gyro readings by this factor
  // calibrates the readings to show angular velocity
  // in radians/sec.
  const double GYRO_SCALE_FACTOR_ = 0.001055997;
};

#endif  // WIIMOTE_WIIMOTE_CONTROLLER_H
