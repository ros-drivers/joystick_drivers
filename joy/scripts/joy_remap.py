#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from sensor_msgs.msg import Joy


class JoyRemap(object):
    def __init__(self):
        self.mappings = self.load_mappings("~mappings")
        self.warn_remap("joy_out")
        self.pub = rospy.Publisher(
            "joy_out", Joy, queue_size=1)
        self.warn_remap("joy_in")
        self.sub = rospy.Subscriber(
            "joy_in", Joy, self.callback,
            queue_size=rospy.get_param("~queue_size", None))

    def load_mappings(self, ns):
        return {
            "buttons": rospy.get_param(ns + "/buttons", []),
            "axes": rospy.get_param(ns + "/axes", []),
        }

    def warn_remap(self, name):
        if name == rospy.remap_name(name):
            rospy.logwarn("topic '%s' is not remapped" % name)

    def callback(self, in_msg):
        out_msg = Joy(header=in_msg.header)
        map_axes = self.mappings["axes"]
        map_btns = self.mappings["buttons"]
        out_msg.axes = [0.0] * len(map_axes)
        out_msg.buttons = [0] * len(map_btns)
        in_dic = {"axes": in_msg.axes, "buttons": in_msg.buttons}
        for i, exp in enumerate(map_axes):
            try:
                out_msg.axes[i] = eval("{}".format(exp), {}, in_dic)
            except NameError as e:
                rospy.logerr("You are using vars other than 'buttons' or 'axes': %s" % e)
            except UnboundLocalError as e:
                rospy.logerr("Wrong form: %s" % e)
            except Exception as e:
                raise e

        for i, exp in enumerate(map_btns):
            try:
                if eval("{}".format(exp), {}, in_dic) > 0:
                    out_msg.buttons[i] = 1
            except NameError as e:
                rospy.logerr("You are using vars other than 'buttons' or 'axes': %s" % e)
            except UnboundLocalError as e:
                rospy.logerr("Wrong form: %s" % e)
            except Exception as e:
                raise e

        self.pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node("joy_remap")
    n = JoyRemap()
    rospy.spin()
