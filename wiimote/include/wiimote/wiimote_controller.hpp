// Copyright 2020 Intel Corporation
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

/*
 * ROS Node for interfacing with a wiimote control unit.
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
#ifndef WIIMOTE__WIIMOTE_CONTROLLER_HPP_
#define WIIMOTE__WIIMOTE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <wiimote_msgs/msg/state.hpp>

#include "bluetooth/bluetooth.h"
namespace wiimote_c
{
#include "cwiid.h"  // NOLINT, cpplint wants us to have a directory
}

#include "wiimote/stat_vector_3d.hpp"

#define zeroedByCal(raw, zero, one) (((raw - zero) * 1.0) / ((one - zero) * 1.0))

class WiimoteNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * \brief rclcpp component-compatible constructor
   * \param options
   */
  explicit WiimoteNode(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  char * get_bluetooth_addr();
  void set_bluetooth_addr(const char * bt_str);
  bool pair_wiimote(int flags, int timeout);
  int unpair_wiimote();

  void publish();
  bool wiimote_is_connected();
  void check_connection();

  void set_led_state(uint8_t led_state);
  void set_rumble_state(uint8_t rumble);

private:
  void set_report_mode(uint8_t rpt_mode);
  void check_factory_calibration_data();
  void reset_motion_plus_state();
  void reset_nunchuck_state();
  void reset_classic_state();

  static wiimote_c::cwiid_err_t cwiid_error_callback;

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
  void joy_set_feedback_callback(sensor_msgs::msg::JoyFeedbackArray::ConstSharedPtr feedback);
  bool service_imu_calibrate_callback(
    std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr);

  bool is_collecting_wiimote();
  bool is_collecting_nunchuk();
  bool is_collecting_classic();
  bool is_collecting_motionplus();

  bool is_present_nunchuk();
  bool is_present_classic();
  bool is_present_motionplus();

  bool calibrate_joystick(uint8_t * stick, uint8_t(&center)[2], const char * name);
  void update_joystick_min_max(
    uint8_t * stick, uint8_t(&stick_min)[2], uint8_t(&stick_max)[2], const char * name);
  void calculate_joystick_axis_xy(
    uint8_t * stick_current, uint8_t * stick_min, uint8_t * stick_max, uint8_t * stick_center,
    double (& stick)[2]);

  void publish_joy();
  void publish_imu_data();
  void publish_wiimote_state();
  bool publish_wiimote_nunchuk_common();
  void publish_wiimote_nunchuk();
  void publish_wiimote_classic();

  void initialize_wiimote_state();
  bool get_state_sample();

  void set_led_bit(uint8_t led, bool on);
  void set_rumble_bit(bool on);

  rclcpp::Logger logger_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub_;
  rclcpp_lifecycle::LifecyclePublisher<wiimote_msgs::msg::State>::SharedPtr wiimote_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Joy>::SharedPtr wiimote_nunchuk_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Joy>::SharedPtr wiimote_classic_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr imu_is_calibrated_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JoyFeedbackArray>::SharedPtr joy_set_feedback_sub_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr imu_calibrate_srv_;

  rclcpp::TimerBase::SharedPtr check_connection_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // bluetooth device address
  bdaddr_t bt_device_addr_;

  // wiimote handle
  wiimote_c::cwiid_wiimote_t * wiimote_;

  // Last state of the Wiimote
  struct wiimote_c::cwiid_state wiimote_state_;
  // Time last state sample was taken
  uint32_t state_secs_;
  uint32_t state_nsecs_;

  // Which data items should be reported in state
  uint8_t report_mode_;

  // Calibration status Wiimote
  struct wiimote_c::acc_cal wiimote_cal_;  // wiimote acceleration factory calibration
  bool wiimote_calibrated_;

  rclcpp::Time calibration_time_;

  // Joystick related constants
  const uint8_t JOYSTICK_NUNCHUK_DEFAULT_CENTER_ = 127;  // Theoretical center
  const uint8_t JOYSTICK_NUNCHUK_20PERCENT_MAX_ = 205;
  const uint8_t JOYSTICK_NUNCHUK_20PERCENT_MIN_ = 50;
  const uint8_t JOYSTICK_CLASSIC_LEFT_DEFAULT_CENTER_ = 31;  // Theoretical center
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
  uint8_t classic_stick_left_center_[2];  // nunchuk stick center position
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
  std::array<double, 9> linear_acceleration_covariance_;
  std::array<double, 9> angular_velocity_covariance_;

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

#endif  // WIIMOTE__WIIMOTE_CONTROLLER_HPP_
