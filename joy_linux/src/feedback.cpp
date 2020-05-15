/*
 * Copyright (c) 2020, Bundesanstalt für Materialforschung und -prüfung (BAM).
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
 *     * Neither the name of the copyright holder nor the names of its
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

#include <joy_linux/feedback.hpp>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <functional>
#include <memory>
#include <string>

FeedbackDevice::FeedbackDevice(std::shared_ptr<rclcpp::Node> node, std::string device)
  : node_(node)
  , device_(device)
  , fd_(-1)
  , update_feedback_(false)
  , feedback_sub_(node_->create_subscription<sensor_msgs::msg::JoyFeedbackArray>(
                    "joy/set_feedback",
                    rclcpp::QoS(10),
                    std::bind(&FeedbackDevice::setFeedback, this,
                              std::placeholders::_1)))
{}

FeedbackDevice::~FeedbackDevice()
{
  close();
}

void FeedbackDevice::open()
{
  if (!device_.empty()) {
    fd_ = ::open(device_.c_str(), O_RDWR);
    if (fd_ == -1) {
      RCLCPP_ERROR(
        node_->get_logger(), "Couldn't open joystick force feedback: %s", strerror(errno));
      return;
    }

    /* Set the gain of the device*/
    int gain = 100;           /* between 0 and 100 */
    struct input_event ie;      /* structure used to communicate with the driver */

    ie.type = EV_FF;
    ie.code = FF_GAIN;
    ie.value = 0xFFFFUL * gain / 100;

    if (::write(fd_, &ie, sizeof(ie)) == -1) {
      RCLCPP_ERROR(
        node_->get_logger(), "Couldn't open joystick force feedback: %s", strerror(errno));
      return;
    }

    joy_effect_.id = -1;
    joy_effect_.direction = 0;  // down
    joy_effect_.type = FF_RUMBLE;
    joy_effect_.u.rumble.strong_magnitude = 0;
    joy_effect_.u.rumble.weak_magnitude = 0;
    joy_effect_.replay.length = 1000;
    joy_effect_.replay.delay = 0;

    // upload the effect
    // FIXME: check the return value here
    ::ioctl(fd_, EVIOCSFF, &joy_effect_);
  }
}

void FeedbackDevice::close()
{
  if (fd_ != -1) {
    ::close(fd_);
    fd_ = -1;
  }
}

void FeedbackDevice::uploadEffect()
{
  if (fd_ != -1) {
    struct input_event start;
    start.type = EV_FF;
    start.code = joy_effect_.id;
    start.value = 3;
    if (::write(fd_, (const void *) &start, sizeof(start)) == -1) {
      RCLCPP_ERROR(node_->get_logger(), "Lost connection to feedback device");
      close();
      return;
    }

    // upload the effect
    if (update_feedback_ == true) {
      // FIXME: check the return value here.
      ::ioctl(fd_, EVIOCSFF, &joy_effect_);
      update_feedback_ = false;
    }
  }
}

void FeedbackDevice::setFeedback(const std::shared_ptr<sensor_msgs::msg::JoyFeedbackArray> msg)
{
  if (fd_ == -1) {
    return;  // we arent ready yet
  }

  size_t size = msg->array.size();
  for (size_t i = 0; i < size; i++) {
    // process each feedback
    if (msg->array[i].type == 1) {  // TYPE_RUMBLE
      // if id is zero, thats low freq, 1 is high
      joy_effect_.direction = 0;  // down
      joy_effect_.type = FF_RUMBLE;
      if (msg->array[i].id == 0) {
        joy_effect_.u.rumble.strong_magnitude =
          (static_cast<float>(1 << 15)) * msg->array[i].intensity;
      } else {
        joy_effect_.u.rumble.weak_magnitude =
          (static_cast<float>(1 << 15)) * msg->array[i].intensity;
      }

      joy_effect_.replay.length = 1000;
      joy_effect_.replay.delay = 0;

      update_feedback_ = true;
    }
  }
}

