#!/usr/bin/python
################################################################################
#
# File:         wiimode_node.py
# RCS:          $Header: $
# Description:  Top level ROS node that publishes Wiimote data
#               and allows Wiimote rumble/LED setting.
# Author:       Andreas Paepcke
# Created:      Thu Sep 10 10:31:44 2009
# Modified:     Thu Sep 10 10:33:02 2009 (Andreas Paepcke) paepcke@anw.willowgarage.com
# Language:     Python
# Package:      N/A
# Status:       Experimental (Do Not Distribute)
#
# (c) Copyright 2009, Willow Garage, all rights reserved.
#
################################################################################

#!/usr/bin/env python

# Python Standard Modules:
import threading

# ROS-Related Modules:
import roslib; roslib.load_manifest('wiimote')
import rospy
from msg import wiimote_accel, wiimote_gyro
from rospy.numpy_msg import numpy_msg   # For numpy arrays in Wiimote messages

# WIIMote Modules:
from wiimoteExceptions import *
import WIIMote

def runWiimoteNode():
    """Initialize the wiimote_node, establishing its name for communication with the Master"""

    rospy.init_node('wiimote', anonymous=True)
    commonLock = threading.Lock()
    try:
        wiimote = WIIMote.WIIMote()
        wiimote.calibrate()
        AcceleratorSender(commonLock, wiimote, freq=1).start()
        GyroSender(commonLock, wiimote, freq=1).start()
    except WiimoteError, e:
        rospy.loginfo(str(e))
        exit()

class AcceleratorSender(threading.Thread):
    """Broadcasting Wiimote accelerator readings to Topic wiimote_accel"""
    
    def __init__(self, wiiLock, wiiMote, freq=100):
        """Initializes the Wiimote accelerator publisher.
    
        Parameters:
            wiiLock: a threading.Lock object that will be used by the accelerator data sender,
                     the gyro data sender, and the Wiimote driver to stay out of each
                     others' hair.
            wiiMote: a bluetooth-connected, calibrated WIIMote instance
            freq:    the message sending frequency in messages/sec. Max is 100, because
                     the Wiimote only samples the sensors at 100Hz.
        """
        
        threading.Thread.__init__(self)
        self.wiiLock = wiiLock
        self.wiiMote = wiiMote
        self.freq = freq
        self.sleepDuration = 1.0 / freq
        
        self.pub = rospy.Publisher('wiimote_accel', numpy_msg(wiimote_accel))        
        
    def run(self):
        """Loop that obtains the latest wiimote state, publishes the accelerometer data, and sleeps"""
        
        while True:
            self.wiiLock.acquire()
            wiistate = self.wiiMote.wiiMoteState
            self.wiiLock.release()
            if wiistate is None or wiistate.acc is None: continue
            
            msg = wiimote_accel(header=None, accel=wiistate.acc.tuple())
            measureTime = wiistate.time
            timeSecs = int(measureTime)
            timeNSecs = int(abs(timeSecs - measureTime) * 10**9)
            msg.header.stamp.secs = timeSecs
            msg.header.stamp.nsecs = timeNSecs
            
            self.pub.publish(msg.header, msg.accel)
            
            rospy.loginfo(str(wiistate.acc))
            rospy.sleep(self.sleepDuration)
            
class GyroSender(threading.Thread):
    """Broadcasting Wiimote gyro (angular rate) readings to Topic wiimote_gyro"""
    
    def __init__(self, wiiLock, wiiMote, freq=100):
        """Initializes the Wiimote gyro publisher.
    
        Parameters:
            wiiLock: a threading.Lock object that will be used by the accelerator data sender,
                     the gyro data sender, and the Wiimote driver to stay out of each
                     others' hair.
            wiiMote: a bluetooth-connected, calibrated WIIMote instance
            freq:    the message sending frequency in messages/sec. Max is 100, because
                     the Wiimote only samples the sensors at 100Hz.
        """
        
        threading.Thread.__init__(self)
        self.wiiLock = wiiLock
        self.wiiMote = wiiMote
        self.freq = freq
        self.sleepDuration = 1.0 / freq
        
        self.pub = rospy.Publisher('wiimote_gyro', numpy_msg(wiimote_gyro))        
        
    def run(self):
        """Loop that obtains the latest wiimote state, publishes the gyro (angular velocity) data, and sleeps"""
        
        while True:
            self.wiiLock.acquire()
            wiistate = self.wiiMote.wiiMoteState
            self.wiiLock.release()
            if wiistate is None or wiistate.acc is None: continue
            
            msg = wiimote_gyro(header=None, angular_velocity=wiistate.angleRate.tuple())
            measureTime = wiistate.time
            timeSecs = int(measureTime)
            timeNSecs = int(abs(timeSecs - measureTime) * 10**9)
            msg.header.stamp.secs = timeSecs
            msg.header.stamp.nsecs = timeNSecs
            
            self.pub.publish(msg.header, msg.angular_velocity)
            
            rospy.loginfo(str(wiistate.angleRate))
            rospy.sleep(self.sleepDuration)
            
        
if __name__ == '__main__':
    try:
        runWiimoteNode()
    except Exception, e:
        print(str(e))
