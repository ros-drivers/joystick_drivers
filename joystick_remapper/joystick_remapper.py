#!/usr/bin/python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

##\author Blaise Gassend
##\brief Remaps joystick buttons to allow different joystick to command a single node

import roslib
roslib.load_manifest('joystick_remapper')
import rospy
from joy.msg import Joy

extract_warn = False

def extract(list, index):
    global extarct_warn
    if index == -1:
        return 0

    try:
        return list[index]
    except:
        extract_warn = True
        return 0

def remap(src, mapping):
    if mapping == None:
        return src
    else:
        return [extract(src, i) for i in mapping]

def str_mapping(mapping):
    if mapping == None:
        return "<identity>"
    else:
        return "".join("%i->%i "%(mapping[i], i) for i in range(0, len(mapping)))

##\brief Subscribes to input, maps and publishes to output topic
class Remapper:
    def __init__(self, button_mapping, axis_mapping):
        rospy.Subscriber("joy_source", Joy, self.callback)
        self.pub = rospy.Publisher("joy_dest", Joy)
        check_remapped("joy_source")
        check_remapped("joy_dest")
        self.button_mapping = button_mapping
        self.axis_mapping = axis_mapping
        self.warned = False
        rospy.loginfo("Starting joystick remapping")
        rospy.loginfo("Button mapping: %s"%str_mapping(self.button_mapping))
        rospy.loginfo("Axis mapping: %s"%str_mapping(self.axis_mapping))

    def callback(self, inmsg):
        global extract_warn
        outmsg = Joy()
        outmsg.buttons = remap(inmsg.buttons, self.button_mapping)
        outmsg.axes = remap(inmsg.axes, self.axis_mapping)
        self.pub.publish(outmsg)
        if not self.warned and extract_warn:
            self.warned = True
            rospy.logwarn("Out of range remapping. Setting to zero. Joystick has %i buttons and %i axes.")%(len(inmsg.buttons), len(inmsg.axes))

##\brief Gives parameters as a list. Ex. input: "0 2 8"
def get_param_list(name):
    try:
        s = str(rospy.get_param(name));
        if s == "=":
            return None # Identity
        return map(int, s.split())
    except KeyError:
        rospy.logwarn("No %s parameter found. Using identity mapping."%name)
        return None
    except ValueError:
        rospy.logfatal("Parameter '%s' contains a non-integer element. Aborting."%name)
        exit(-1)

def check_remapped(name):
    if name == rospy.remap_name(name):
        rospy.logerr("Topic %s was not remapped. This is probably unintentional."%name)

if __name__ == "__main__":
    try:
        rospy.init_node('joystick_remapper', log_level=rospy.DEBUG)
        button_mapping = get_param_list('~button_mapping')
        axis_mapping = get_param_list('~axis_mapping')
        Remapper(button_mapping, axis_mapping)
        while not rospy.is_shutdown():
            rospy.sleep(1)
    except KeyboardInterrupt:
        pass
    except rospy.exceptions.ROSInterruptException:
        pass


