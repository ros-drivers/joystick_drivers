#!/usr/bin/env python3

# Copyright (c) 2020, Intel Corporation.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.


import rclpy
import rclpy.exceptions
from sensor_msgs.msg import JoyFeedbackArray
from sensor_msgs.msg import JoyFeedback
import time

INTER_PATTERN_SLEEP_DURATION = 0.2


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(node_name='ledControlTester')
    pub = node.create_publisher(msg_type=JoyFeedbackArray, topic='/joy/set_feedback', qos_profile=1)

    led0 = JoyFeedback()
    led0.type = JoyFeedback.TYPE_LED
    led0.id = 0
    led1 = JoyFeedback()
    led1.type = JoyFeedback.TYPE_LED
    led1.id = 1
    led2 = JoyFeedback()
    led2.type = JoyFeedback.TYPE_LED
    led2.id = 2
    led3 = JoyFeedback()
    led3.type = JoyFeedback.TYPE_LED
    led3.id = 3
    rum = JoyFeedback()
    rum.type = JoyFeedback.TYPE_RUMBLE
    rum.id = 0

    while rclpy.ok():
        msg = JoyFeedbackArray()
        msg.array = [led0, led1, led2, led3, rum]

        led0.intensity = 0.2
        led3.intensity = 0.2
        rum.intensity = 0.49

        if msg is not None:
            node.get_logger().debug("Msg: " + str(msg))
            pub.publish(msg)
            time.sleep(INTER_PATTERN_SLEEP_DURATION)

        led0.intensity = 1.0
        rum.intensity = 0.51

        if msg is not None:
            node.get_logger().debug("Msg: " + str(msg))
            pub.publish(msg)
            time.sleep(INTER_PATTERN_SLEEP_DURATION)

        led0.intensity = 0.0
        led1.intensity = 1.0
        rum.intensity = 0.0

        if msg is not None:
            node.get_logger().debug("Msg: " + str(msg))
            pub.publish(msg)
            time.sleep(INTER_PATTERN_SLEEP_DURATION)

        led1.intensity = 0.0
        led2.intensity = 1.0
        rum.intensity = 0.7

        if msg is not None:
            node.get_logger().debug("Msg: " + str(msg))
            pub.publish(msg)
            time.sleep(INTER_PATTERN_SLEEP_DURATION)

        led2.intensity = 0.0
        led3.intensity = 1.0
        rum.intensity = 0.49

        if msg is not None:
            node.get_logger().debug("Msg: " + str(msg))
            pub.publish(msg)
            time.sleep(INTER_PATTERN_SLEEP_DURATION)

        led1.intensity = 1.0
        led2.intensity = 1.0
        rum.intensity = 1.0

        if msg is not None:
            node.get_logger().debug("Msg: " + str(msg))
            pub.publish(msg)
            time.sleep(INTER_PATTERN_SLEEP_DURATION)

        led0.intensity = 1.0
        led1.intensity = 0.4
        led2.intensity = 0.4

        msg.array = [led0, led1, led2]

        if msg is not None:
            node.get_logger().debug("Msg: " + str(msg))
            pub.publish(msg)
            time.sleep(INTER_PATTERN_SLEEP_DURATION)


if __name__ == '__main__':

    print("\n   ****************************************************************\n")
    print("****     You should see six LED on/off configurations, and feel Rumbles!  ****")
    print("\n   **************************************************************")
    print("[off, off, off, off]")
    print("[on,  off, off, off]")
    print("[off, on,  off, off]")
    print("[off, off, on,  off]")
    print("[off, off, off, on ]")
    print("[off, on,  on,  on ]")
    print("[on,  off, off, on ]")
    try:
        main()
    except rclpy.exceptions.ROSInterruptException:
        pass
