// Author: Hye-jong KIM

#include "spacenav_node/spacenav_node.hpp"


namespace spacenav_node
{

SpacenavNode::SpacenavNode(const rclcpp::NodeOptions & options)
: Node("spacenav", options)
{
  RCLCPP_INFO(get_logger(), "Space Nvigation node.");
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::QoS qos(rclcpp::KeepLast(7));
  offset_pub_ = this->create_publisher<Vector3>(
    "spacenav/offset",
    rclcpp::SystemDefaultsQoS());
  rot_offset_pub_ = this->create_publisher<Vector3>(
    "spacenav/rot_offset",
    rclcpp::SystemDefaultsQoS());
  twist_pub_ = this->create_publisher<Twist>(
    "spacenav/twist",
    rclcpp::SystemDefaultsQoS());
  joy_pub_ = this->create_publisher<Joy>(
    "spacenav/joy",
    rclcpp::SystemDefaultsQoS());

  // Used to scale joystick output to be in [-1, 1]. Estimated from data, and not necessarily correct.
  this->declare_parameter<double>("full_scale", 512);
  full_scale_ = this->get_parameter("full_scale").as_double();
  if (full_scale_ < 1e-10) {
    full_scale_ = 512;
  }

  // Scale factors for the different axes. End output will be within [-scale, +scale], provided
  // full_scale normalizes to within [-1, 1].
  this->declare_parameter<std::vector<double>>("linear_scale", std::vector<double>(3, 1));
  linear_scale_ = this->get_parameter("linear_scale").as_double_array();
  this->declare_parameter<std::vector<double>>("angular_scale", std::vector<double>(3, 1));
  angular_scale_ = this->get_parameter("angular_scale").as_double_array();

  if (!ensureThreeComponents(linear_scale_)) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Parameter " << this->get_namespace() <<
        "/linear_scale must have either one or three components.");
    exit(EXIT_FAILURE);
  }
  if (!ensureThreeComponents(angular_scale_)) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Parameter " << this->get_namespace() <<
        "/angular_scale must have either one or three components.");
    exit(EXIT_FAILURE);
  }
  RCLCPP_DEBUG(
    get_logger(), "linear_scale: %.3f %.3f %.3f",
    linear_scale_[0], linear_scale_[1], linear_scale_[2]);
  RCLCPP_DEBUG(
    get_logger(), "angular_scale: %.3f %.3f %.3f",
    angular_scale_[0], angular_scale_[1], angular_scale_[2]);

  if (spnav_open() == -1) {
    RCLCPP_ERROR(
      get_logger(), "Could not open the space navigator device. "
      "Did you remember to run spacenavd (as root)?");
    exit(EXIT_FAILURE);
  }

  // Parameters
  // The number of polls needed to be done before the device is considered "static"
  this->declare_parameter<int>("static_count_threshold", 30);
  static_count_threshold_ = this->get_parameter("static_count_threshold").as_int();

  // If true, the node will zero the output when the device is "static"
  this->declare_parameter<bool>("zero_when_static", true);
  zero_when_static_ = this->get_parameter("zero_when_static").as_bool();

  // If the device is considered "static" and each trans, rot normed component
  // is below the deadband, it will output zeros in either rotation,
  // translation, or both.
  this->declare_parameter<double>("static_trans_deadband", true);
  static_trans_deadband_ = this->get_parameter("static_trans_deadband").as_double();
  this->declare_parameter<double>("static_rot_deadband", true);
  static_rot_deadband_ = this->get_parameter("static_rot_deadband").as_double();

  joystick_msg_.axes.resize(6);
  joystick_msg_.buttons.resize(2);

  joy_stale_ = false;
  queue_empty_ = true;

  timer_ = this->create_wall_timer(
    1ms, [this]() -> void {
      while (1) {
        joy_stale_ = false;
        queue_empty_ = false;

        // Sleep when the queue is empty.
        // If the queue is empty 30 times in a row output zeros.
        // Output changes each time a button event happens, or when a motion
        // event happens and the queue is empty.
        joystick_msg_.header.stamp = rclcpp::Clock().now();

        switch (spnav_poll_event(&sev_)) {
          case 0:
            queue_empty_ = true;

            if (++no_motion_count_ > static_count_threshold_) {
              if (zero_when_static_ ||
              (fabs(normed_vx_) < static_trans_deadband_ &&
              fabs(normed_vy_) < static_trans_deadband_ &&
              fabs(normed_vz_) < static_trans_deadband_))
              {
                normed_vx_ = normed_vy_ = normed_vz_ = 0;
              }

              if (zero_when_static_ ||
              (fabs(normed_wx_) < static_rot_deadband_ &&
              fabs(normed_wy_) < static_rot_deadband_ &&
              fabs(normed_wz_) < static_rot_deadband_))
              {
                normed_wx_ = normed_wy_ = normed_wz_ = 0;
              }

              no_motion_count_ = 0;
              motion_stale_ = true;
            }
            break;

          case SPNAV_EVENT_MOTION:
            normed_vx_ = sev_.motion.z / full_scale_;
            normed_vy_ = -sev_.motion.x / full_scale_;
            normed_vz_ = sev_.motion.y / full_scale_;

            normed_wx_ = sev_.motion.rz / full_scale_;
            normed_wy_ = -sev_.motion.rx / full_scale_;
            normed_wz_ = sev_.motion.ry / full_scale_;

            motion_stale_ = true;
            break;

          case SPNAV_EVENT_BUTTON:
            joystick_msg_.buttons[sev_.button.bnum] = sev_.button.press;
            joy_stale_ = true;
            break;

          default:
            RCLCPP_WARN(
              this->get_logger(),
              "Unknown message type in spacenav. This should never happen.");
            break;
        }
        if (motion_stale_ && (queue_empty_ || joy_stale_)) {
          // The offset and rot_offset are scaled.
          Vector3 offset_msg;
          offset_msg.x = normed_vx_ * linear_scale_[0];
          offset_msg.y = normed_vy_ * linear_scale_[1];
          offset_msg.z = normed_vz_ * linear_scale_[2];
          offset_pub_->publish(offset_msg);

          Vector3 rot_offset_msg;
          rot_offset_msg.x = normed_wx_ * angular_scale_[0];
          rot_offset_msg.y = normed_wy_ * angular_scale_[1];
          rot_offset_msg.z = normed_wz_ * angular_scale_[2];
          rot_offset_pub_->publish(rot_offset_msg);

          Twist twist_msg;
          twist_msg.linear = offset_msg;
          twist_msg.angular = rot_offset_msg;
          twist_pub_->publish(twist_msg);

          // The joystick.axes are normalized within [-1, 1].
          joystick_msg_.axes[0] = static_cast<float>(normed_vx_);
          joystick_msg_.axes[1] = static_cast<float>(normed_vy_);
          joystick_msg_.axes[2] = static_cast<float>(normed_vz_);
          joystick_msg_.axes[3] = static_cast<float>(normed_wx_);
          joystick_msg_.axes[4] = static_cast<float>(normed_wy_);
          joystick_msg_.axes[5] = static_cast<float>(normed_wz_);

          no_motion_count_ = 0;
          motion_stale_ = false;
          joy_stale_ = true;
        }

        if (joy_stale_) {
          joy_pub_->publish(joystick_msg_);
        }

        if (queue_empty_) {
          usleep(1000);
        }

      }
    });
}

SpacenavNode::~SpacenavNode()
{
  this->undeclare_parameter("full_scale");
  this->undeclare_parameter("linear_scale");
  this->undeclare_parameter("angular_scale");
  this->undeclare_parameter("static_count_threshold");
  this->undeclare_parameter("zero_when_static");
  this->undeclare_parameter("static_trans_deadband");
  this->undeclare_parameter("static_rot_deadband");
  spnav_close();
}

bool SpacenavNode::ensureThreeComponents(std::vector<double> & param)
{
  if (param.size() == 0) {
    param.push_back(1);
    param.push_back(1);
    param.push_back(1);
    return True;
  }
  if (param.size() == 3) {
    return True;
  }
  if (param.size() == 1) {
    param.push_back(param[0]);
    param.push_back(param[0]);
    return True;
  }
  return False;
}

}  // namespace spacenav_node
RCLCPP_COMPONENTS_REGISTER_NODE(spacenav_node::SpacenavNode)
