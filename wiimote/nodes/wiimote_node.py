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

import threading
#**********
import time
#**********

import roslib; roslib.load_manifest('wiimote')
import rospy
#****from msg import wiimote_accel, wiimote_gyro
from rospy.numpy_msg import numpy_msg   # For numpy arrays in Wiimote messages

class AcceleratorSender(threading.Thread):
	"""Broadcasting Wiimote accelerator readings to Topic wiimote_accel"""
	
	def __init__(self, wiiLock, freq=100):
		"""Initializes the Wiimote accelerator publisher.
	
		Parameters:
			wiiLock: a threading.Lock object that will be used by the accelerator data sender,
				 	the gyro data sender, and the Wiimote driver to stay out of each
				 	others' hair.
			freq:    the message sending frequency in messages/sec. Max is 100, because
				     the Wiimote only samples the sensors at 100Hz.
		"""

		#*****pub = rospy.Publisher('wiimote_accel', numpy_msg(wiimote_accel))
		while True:
			print "Accelerator sender active." + time.asctime()
			time.sleep(1)
		
		

def runWiimoteNode():
	"""Initialize the wiimote_node, establishing its name for communication with the Master"""

	rospy.init_node('wiimote', anonymous=True)
	commonLock = threading.Lock()
	AcceleratorSender(commonLock, freq=1).start()
	#GyroSender(commonLock, freq=1).start()
	rospy.spin()


if __name__ == '__main__':
    runWiimoteNode()
