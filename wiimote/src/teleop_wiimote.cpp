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
 * ROS Node for using a wiimote control unit to direct a robot.
 */

#include "wiimote/teleop_wiimote.hpp"

#include <cmath>

namespace
{
// Sane defaults based on the TurtleBot
// TurtleBot maximum speed documented at 25.6"/second ~= 0.65024 m/s
constexpr double kDefaultMaxLinearX = 0.65024;  // m/s
// TurtleBot maximum angular speed is documented at 180 degrees Pi / second
constexpr double kDefaultMaxAngularZ = M_PI;  // rad/s

constexpr double kDefaultPercentLinearThrottle = 0.75;   // decimal
constexpr double kDefaultPercentAngularThrottle = 0.75;  // decimal
}  // namespace

TeleopWiimote::TeleopWiimote(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("teleop_wiimote", options), logger_(get_logger())
{
  RCLCPP_INFO(logger_, "TeleopWiimote lifecycle node created.");
  auto linear_x_max_vel_param_desc = rcl_interfaces::msg::ParameterDescriptor();
  linear_x_max_vel_param_desc.name = "linear.x.max_velocity";
  linear_x_max_vel_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  linear_x_max_vel_param_desc.description = "Maximum linear velocity in m/s";
  declare_parameter("linear.x.max_velocity", kDefaultMaxLinearX, linear_x_max_vel_param_desc);

  auto linear_x_min_vel_param_desc = rcl_interfaces::msg::ParameterDescriptor();
  linear_x_min_vel_param_desc.name = "linear.x.min_velocity";
  linear_x_min_vel_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  linear_x_min_vel_param_desc.description = "Minimum linear velocity in m/s";
  declare_parameter("linear.x.min_velocity", -kDefaultMaxLinearX, linear_x_min_vel_param_desc);

  auto angular_z_max_vel_param_desc = rcl_interfaces::msg::ParameterDescriptor();
  angular_z_max_vel_param_desc.name = "angular.z.max_velocity";
  angular_z_max_vel_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  angular_z_max_vel_param_desc.description = "Maximum angular velocity in rad/s";
  declare_parameter("angular.z.max_velocity", kDefaultMaxAngularZ, angular_z_max_vel_param_desc);

  auto angular_z_min_vel_param_desc = rcl_interfaces::msg::ParameterDescriptor();
  angular_z_min_vel_param_desc.name = "angular.z.min_velocity";
  angular_z_min_vel_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  angular_z_min_vel_param_desc.description = "Minimum angular velocity in rad/s";
  declare_parameter("angular.z.min_velocity", -kDefaultMaxAngularZ, angular_z_min_vel_param_desc);

  auto linear_x_throttle_param_desc = rcl_interfaces::msg::ParameterDescriptor();
  linear_x_throttle_param_desc.name = "linear.x.throttle_percent";
  linear_x_throttle_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  linear_x_throttle_param_desc.description = "Linear x throttle percentage in decimal";
  declare_parameter(
    "linear.x.throttle_percent", kDefaultPercentLinearThrottle, linear_x_throttle_param_desc);

  auto angular_z_throttle_param_desc = rcl_interfaces::msg::ParameterDescriptor();
  angular_z_throttle_param_desc.name = "angular.z.throttle_percent";
  angular_z_throttle_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  angular_z_throttle_param_desc.description = "Angular z throttle percentage in decimal";
  declare_parameter("angular.z.throttle_percent", kDefaultPercentAngularThrottle);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TeleopWiimote::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring: started");
  // ROS2 resources
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 3);
  joy_pub_ = create_publisher<sensor_msgs::msg::JoyFeedbackArray>("joy/set_feedback", 3);
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "wiimote/nunchuk", 10,
    [this](sensor_msgs::msg::Joy::ConstSharedPtr joy) {this->joy_callback(joy);});
  wiimote_sub_ = create_subscription<wiimote_msgs::msg::State>(
    "wiimote/state", 10, [this](wiimote_msgs::msg::State::ConstSharedPtr wiistate) {
      this->wiimote_state_callback(wiistate);
    });

  linear_x_max_velocity_ = get_parameter("linear.x.max_velocity").as_double();
  linear_x_min_velocity_ = get_parameter("linear.x.min_velocity").as_double();
  angular_z_max_velocity_ = get_parameter("angular.z.max_velocity").as_double();
  angular_z_min_velocity_ = get_parameter("angular.z.min_velocity").as_double();
  percent_linear_throttle_ = get_parameter("linear.x.throttle_percent").as_double();
  percent_angular_throttle_ = get_parameter("angular.z.throttle_percent").as_double();

  dpad_in_use_ = false;
  njoy_in_use_ = false;

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TeleopWiimote::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating");
  RCLCPP_INFO(logger_, "Updating configuration variables with latest parameters");
  auto val = get_parameter("linear.x.max_velocity").as_double();
  if (linear_x_max_velocity_ != val) {
    RCLCPP_INFO(
      logger_, "linear.x.max_velocity updated. From: %3.0f, To: %3.0f", linear_x_max_velocity_,
      val);
    linear_x_max_velocity_ = val;
  }

  val = get_parameter("linear.x.min_velocity").as_double();
  if (linear_x_min_velocity_ != val) {
    RCLCPP_INFO(
      logger_, "linear.x.min_velocity updated. From: %3.0f, To: %3.0f", linear_x_min_velocity_,
      val);
    linear_x_min_velocity_ = val;
  }

  val = get_parameter("angular.z.max_velocity").as_double();
  if (angular_z_max_velocity_ != val) {
    RCLCPP_INFO(
      logger_, "angular.z.max_velocity updated. From: %3.0f, To: %3.0f", angular_z_max_velocity_,
      val);
    angular_z_max_velocity_ = val;
  }

  val = get_parameter("angular.z.min_velocity").as_double();
  if (angular_z_min_velocity_ != val) {
    RCLCPP_INFO(
      logger_, "angular.z.min_velocity updated. From: %3.0f, To: %3.0f", angular_z_min_velocity_,
      val);
    angular_z_min_velocity_ = val;
  }

  val = get_parameter("linear.x.throttle_percent").as_double();
  if (percent_linear_throttle_ != val) {
    RCLCPP_INFO(
      logger_, "linear.x.throttle_percent updated. From: %3.0f, To: %3.0f",
      percent_linear_throttle_, val);
    percent_linear_throttle_ = val;
  }

  val = get_parameter("angular.z.throttle_percent").as_double();
  if (percent_angular_throttle_ != val) {
    RCLCPP_INFO(
      logger_, "angular.z.throttle_percent updated. From: %3.0f, To: %3.0f",
      percent_angular_throttle_, val);
    percent_angular_throttle_ = val;
  }

  // Activate lifecycle publishers
  vel_pub_->on_activate();
  joy_pub_->on_activate();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TeleopWiimote::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating");
  vel_pub_->on_deactivate();
  joy_pub_->on_deactivate();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TeleopWiimote::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning Up");
  vel_pub_.reset();
  joy_pub_.reset();
  joy_sub_.reset();
  wiimote_sub_.reset();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TeleopWiimote::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    logger_, "Shutting Down. Previous State: %s, id: %d", previous_state.label().c_str(),
    previous_state.id());

  // Nothing to do if unconfigured
  if (previous_state.id() == 1) {
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  vel_pub_.reset();
  joy_pub_.reset();
  joy_sub_.reset();
  wiimote_sub_.reset();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TeleopWiimote::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    logger_, "Error handling WiimoteNode. Previous State: %s, id: %d",
    previous_state.label().c_str(), previous_state.id());
  return LifecycleNodeInterface::CallbackReturn::FAILURE;
}

void TeleopWiimote::set_led_feedback(double value)
{
  sensor_msgs::msg::JoyFeedbackArray joy_feedback_array;
  sensor_msgs::msg::JoyFeedback fb_led0;
  sensor_msgs::msg::JoyFeedback fb_led1;
  sensor_msgs::msg::JoyFeedback fb_led2;
  sensor_msgs::msg::JoyFeedback fb_led3;

  fb_led0.type = sensor_msgs::msg::JoyFeedback::TYPE_LED;
  fb_led0.id = 0;
  fb_led0.intensity = 0.0;
  fb_led1.type = sensor_msgs::msg::JoyFeedback::TYPE_LED;
  fb_led1.id = 1;
  fb_led1.intensity = 0.0;
  fb_led2.type = sensor_msgs::msg::JoyFeedback::TYPE_LED;
  fb_led2.id = 2;
  fb_led2.intensity = 0.0;
  fb_led3.type = sensor_msgs::msg::JoyFeedback::TYPE_LED;
  fb_led3.id = 3;
  fb_led3.intensity = 0.0;

  if (value > 10.0) {
    fb_led0.intensity = 1.0;
  }
  if (value > 35.0) {
    fb_led1.intensity = 1.0;
  }
  if (value > 60.0) {
    fb_led2.intensity = 1.0;
  }
  if (value > 85.0) {
    fb_led3.intensity = 1.0;
  }

  joy_feedback_array.array.push_back(fb_led0);
  joy_feedback_array.array.push_back(fb_led1);
  joy_feedback_array.array.push_back(fb_led2);
  joy_feedback_array.array.push_back(fb_led3);

  joy_pub_->publish(joy_feedback_array);
}

void TeleopWiimote::rumble_feedback(std::chrono::milliseconds duration)
{
  sensor_msgs::msg::JoyFeedbackArray joy_feedback_array;
  sensor_msgs::msg::JoyFeedback fb_rumble;

  fb_rumble.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
  fb_rumble.id = 0;

  fb_rumble.intensity = 1.0;

  joy_feedback_array.array.push_back(fb_rumble);

  joy_pub_->publish(joy_feedback_array);
  std::this_thread::sleep_for(duration);
  fb_rumble.intensity = 0.0;

  joy_feedback_array.array.push_back(fb_rumble);

  joy_pub_->publish(joy_feedback_array);
}

void TeleopWiimote::joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr & joy)
{
  if (dpad_in_use_) {
    return;
  }

  geometry_msgs::msg::Twist vel;

  static constexpr int MSG_BTN_Z = 0;
  static constexpr int MSG_BTN_C = 1;

  double x = joy->axes[0];
  double y = joy->axes[1];
  static constexpr double abs_error = 0.000001;

  if (fabs(x) > abs_error || fabs(y) > abs_error) {
    njoy_in_use_ = true;

    double boost = 1.0;

    RCLCPP_DEBUG(logger_, "nunchuk: x: %f, y: %f", x, y);

    if (joy->buttons[MSG_BTN_Z] || joy->buttons[MSG_BTN_C]) {
      RCLCPP_DEBUG(
        logger_, "buttons[]: Z: %d, C: %d", joy->buttons[MSG_BTN_Z], joy->buttons[MSG_BTN_C]);

      // Z-Button is thrusters on!
      if (joy->buttons[MSG_BTN_Z]) {
        boost = 2.0;
      }

      // C-Button is easy does it.
      if (joy->buttons[MSG_BTN_C]) {
        boost = 0.25;
      }
    }

    if (y >= 0.0) {
      vel.linear.x = fmin(
        (y * boost * (linear_x_max_velocity_ * percent_linear_throttle_)), linear_x_max_velocity_);

      if (x >= 0.0) {
        vel.angular.z = fmin(
          (x * boost * (angular_z_max_velocity_ * percent_angular_throttle_)),
          angular_z_max_velocity_);
      } else {
        vel.angular.z = fmax(
          (fabs(x) * boost * (angular_z_min_velocity_ * percent_angular_throttle_)),
          angular_z_min_velocity_);
      }
    } else {
      vel.linear.x = fmax(
        (fabs(y) * boost * (linear_x_min_velocity_ * percent_linear_throttle_)),
        linear_x_min_velocity_);

      if (x > 0.0) {
        vel.angular.z = fmax(
          (x * boost * (angular_z_min_velocity_ * percent_angular_throttle_)),
          angular_z_min_velocity_);
      } else {
        vel.angular.z = fmin(
          (fabs(x) * boost * (angular_z_max_velocity_ * percent_angular_throttle_)),
          angular_z_max_velocity_);
      }
    }

    // In order to spin-in-place left or right, we need full angular with NO linear component.
    // To enable this, we will publish no linear motion if nunchuk joystick Y value is
    // "really small" as the joy stick isn't 100 accurate.
    if (fabs(y) < 0.01) {
      vel.linear.x = 0;
    }

    vel_pub_->publish(vel);
  } else {
    if (njoy_in_use_) {
      vel_pub_->publish(vel);

      njoy_in_use_ = false;
    }
  }
}

void TeleopWiimote::wiimote_state_callback(
  const wiimote_msgs::msg::State::ConstSharedPtr & wiistate)
{
  using namespace std::chrono_literals;

  static constexpr int MSG_BTN_1 = 0;
  static constexpr int MSG_BTN_2 = 1;
  static constexpr int MSG_BTN_PLUS = 2;
  static constexpr int MSG_BTN_MINUS = 3;
  static constexpr int MSG_BTN_A = 4;
  static constexpr int MSG_BTN_B = 5;
  static constexpr int MSG_BTN_UP = 6;
  static constexpr int MSG_BTN_DOWN = 7;
  static constexpr int MSG_BTN_LEFT = 8;
  static constexpr int MSG_BTN_RIGHT = 9;
  static constexpr int MSG_BTN_HOME = 10;

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
  if (wiistate->buttons[MSG_BTN_1]) {
    if (wiistate->buttons[MSG_BTN_PLUS]) {
      if (!plus_depressed) {
        percent_linear_throttle_ += 0.05;
        if (percent_linear_throttle_ >= 1.0) {
          rumble_feedback(100ms);
        }
        percent_linear_throttle_ = fmin(percent_linear_throttle_, 1.0);
        plus_depressed = true;

        set_led_feedback(percent_linear_throttle_ * 100.0);

        set_parameter({"linear.x.throttle_percent", percent_linear_throttle_});
        RCLCPP_INFO(logger_, "Linear X Throttle Percent: %3.0f", percent_linear_throttle_ * 100.0);
      }
    } else {
      plus_depressed = false;

      if (wiistate->buttons[MSG_BTN_MINUS]) {
        if (!minus_depressed) {
          percent_linear_throttle_ -= 0.05;
          if (percent_linear_throttle_ <= 0.1) {
            rumble_feedback(100ms);
          }
          percent_linear_throttle_ = fmax(percent_linear_throttle_, 0.1);
          minus_depressed = true;

          set_led_feedback(percent_linear_throttle_ * 100.0);

          set_parameter({"linear.x.throttle_percent", percent_linear_throttle_});
          RCLCPP_INFO(
            logger_, "Linear X Throttle Percent: %3.0f", percent_linear_throttle_ * 100.0);
        }
      } else {
        minus_depressed = false;
      }
    }

    if (!one_depressed) {
      set_led_feedback(percent_linear_throttle_ * 100.0);

      set_parameter({"linear.x.throttle_percent", percent_linear_throttle_});
      RCLCPP_INFO(logger_, "Linear X Throttle Percent: %3.0f", percent_linear_throttle_ * 100.0);

      one_depressed = true;
    }
  } else if (wiistate->buttons[MSG_BTN_2]) {
    // 2-Button used to set the amount of Angular Throttle
    // Same function and feedbacks as 1-Button (see above)
    if (wiistate->buttons[MSG_BTN_PLUS]) {
      if (!plus_depressed) {
        percent_angular_throttle_ += 0.05;
        if (percent_angular_throttle_ >= 1.0) {
          rumble_feedback(100ms);
        }
        percent_angular_throttle_ = fmin(percent_angular_throttle_, 1.0);
        plus_depressed = true;

        set_led_feedback(percent_angular_throttle_ * 100.0);

        set_parameter({"angular.z.throttle_percent", percent_linear_throttle_});
        RCLCPP_INFO(
          logger_, "Angular Z Throttle Percent: %3.0f", percent_angular_throttle_ * 100.0);
      }
    } else {
      plus_depressed = false;

      if (wiistate->buttons[MSG_BTN_MINUS]) {
        if (!minus_depressed) {
          percent_angular_throttle_ -= 0.05;
          if (percent_angular_throttle_ <= 0.1) {
            rumble_feedback(100ms);
          }
          percent_angular_throttle_ = fmax(percent_angular_throttle_, 0.1);
          minus_depressed = true;

          set_led_feedback(percent_angular_throttle_ * 100.0);

          set_parameter({"angular.z.throttle_percent", percent_linear_throttle_});
          RCLCPP_INFO(
            logger_, "Angular Z Throttle Percent: %3.0f", percent_angular_throttle_ * 100.0);
        }
      } else {
        minus_depressed = false;
      }
    }

    if (!two_depressed) {
      set_led_feedback(percent_angular_throttle_ * 100.0);

      set_parameter({"angular.z.throttle_percent", percent_linear_throttle_});
      RCLCPP_INFO(logger_, "Angular Z Throttle Percent: %3.0f", percent_angular_throttle_ * 100.0);

      two_depressed = true;
    }
  } else {
    if (one_depressed || two_depressed) {
      set_led_feedback(0.0);
    }

    one_depressed = false;
    two_depressed = false;

    // Home-Button used the Wiimote LEDs (see set_led_feedback for levels).
    // to show the approx battery level of the Wiimote.
    // Only works if the 1-Button or 2-Button are not in use.
    if (wiistate->buttons[MSG_BTN_HOME]) {
      if (!home_depressed) {
        RCLCPP_INFO(
          logger_, "Battery[]: raw: %f, percent: %f", wiistate->raw_battery,
          wiistate->percent_battery);
        set_led_feedback(wiistate->percent_battery);
        home_depressed = true;
      }
    } else {
      if (home_depressed) {
        set_led_feedback(0.0);
      }

      home_depressed = false;
    }
  }

  geometry_msgs::msg::Twist vel;

  if (
    !njoy_in_use_ && (wiistate->buttons[MSG_BTN_RIGHT] || wiistate->buttons[MSG_BTN_LEFT] ||
    wiistate->buttons[MSG_BTN_UP] || wiistate->buttons[MSG_BTN_DOWN]))
  {
    dpad_in_use_ = true;

    RCLCPP_DEBUG(
      logger_, "buttons[]: Right: %d, Left: %d, Up: %d, Down: %d, A: %d, B: %d",
      wiistate->buttons[MSG_BTN_RIGHT], wiistate->buttons[MSG_BTN_LEFT],
      wiistate->buttons[MSG_BTN_UP], wiistate->buttons[MSG_BTN_DOWN], wiistate->buttons[MSG_BTN_A],
      wiistate->buttons[MSG_BTN_B]);

    double boost = 1.0;

    // B-Button is thrusters on!
    if (wiistate->buttons[MSG_BTN_B]) {
      boost = 2.0;
    }

    // A-Button is easy does it.
    if (wiistate->buttons[MSG_BTN_A]) {
      boost = 0.25;
    }

    if (wiistate->buttons[MSG_BTN_UP]) {
      vel.linear.x =
        fmin((boost * (linear_x_max_velocity_ * percent_linear_throttle_)), linear_x_max_velocity_);
    } else if (wiistate->buttons[MSG_BTN_DOWN]) {
      vel.linear.x =
        fmax((boost * (linear_x_min_velocity_ * percent_linear_throttle_)), linear_x_min_velocity_);
    }

    if (wiistate->buttons[MSG_BTN_LEFT]) {
      vel.angular.z = fmin(
        (boost * (angular_z_max_velocity_ * percent_angular_throttle_)), angular_z_max_velocity_);
    } else if (wiistate->buttons[MSG_BTN_RIGHT]) {
      vel.angular.z = fmax(
        (boost * (angular_z_min_velocity_ * percent_angular_throttle_)), angular_z_min_velocity_);
    }

    vel_pub_->publish(vel);
  } else {
    if (dpad_in_use_) {
      vel_pub_->publish(vel);
      dpad_in_use_ = false;
    }
  }
}
// Register TeleopWiimote to ros2 components
#include <rclcpp_components/register_node_macro.hpp> //NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(TeleopWiimote)
