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

# TODO: compute IMU msgs
# TODO: compute JOY msgs
# TODO: adjust gyro/accel readings to be correct for IMU/Joy msgs.
# TODO: remove _accel/_gyro msgs

# -------- Python Standard Modules:
import threading

# -------- ROS-Related Modules:
import roslib; roslib.load_manifest('wiimote')
import rospy
# from msg import wiimote
import geometry_msgs.msg
from sensor_msgs.msg import Imu
#***???from joy.msg import Joy
from joy.msg import Joy

# -------- WIIMote Modules:
from wiimoteExceptions import *
from wiimoteConstants import *
import WIIMote

def runWiimoteNode():
    """Initialize the wiimote_node, establishing its name for communication with the Master"""

    rospy.init_node('wiimote_node')
    commonLock = threading.Lock()
    try:
        wiimote = WIIMote.WIIMote()
        wiimote.calibrate()
        IMUSender(commonLock, wiimote, freq=1).start()
        JoySender(commonLock, wiimote, freq=1).start()
    except WiimoteError, e:
        rospy.logdebug(str(e))
        exit()

class WiimoteDataSender(threading.Thread):
    
    def __init__(self, wiiLock, wiiMote, freq=100):
        
        threading.Thread.__init__(self)
        self.wiiLock = wiiLock
        self.wiiMote = wiiMote
        self.freq = freq
        self.sleepDuration = 1.0 / freq
        
        self.linear_acceleration_covariance = [self.wiiMote.varAcc[X], 0., 0.,
                                               0., self.wiiMote.varAcc[Y], 0.,
                                               0., 0.,  self.wiiMote.varAcc[Z]]
        
        self.angular_velocity_covariance = [self.wiiMote.varGyro[X], 0., 0.,
                                            0., self.wiiMote.varGyro[Y], 0.,
                                            0., 0.,  self.wiiMote.varGyro[Z]]
    
    def obtainWiimoteData(self):
        """Retrieve one set of Wiimote measurements from the Wiimote instance. Return scaled accelerator and gyro readings.
        
        We lock self.wiiLock, grab the data, and release the lock.
        Then we canonicalize both accelerator and gyro data through
        scaling them by constants that turn them into m/sec^2, and 
        radians/sec, respectively.
        
        Return: list of canonicalized accelerator and gyro readings. 
        """
        
        while True:
            self.wiiLock.acquire()
            self.wiistate = self.wiiMote.wiiMoteState
            self.wiiLock.release()
            if self.wiistate is not None and self.wiistate.acc is not None: break
            
        return self.canonicalizeWiistate()
        
    def canonicalizeWiistate(self):
        """Scale accelerator and gyro readings to be m/sec^2, and radians/sec, respectively."""
        
        # Convert acceleration, which is in g's into m/sec^@:
        canonicalAccel = self.wiistate.acc.scale(EARTH_GRAVITY)
            
        # Convert gyro reading to radians/sec (see wiimoteConstants.py
        # for origin of this scale factor):
        canonicalAngleRate = self.wiistate.angleRate.scale(GYRO_SCALE_FACTOR)
        
        return [canonicalAccel, canonicalAngleRate]


            
class IMUSender(WiimoteDataSender):
    """Broadcasting Wiimote accelerator and gyro readings as IMU messages to Topic imu_data"""
    
    def __init__(self, wiiLock, wiiMote, freq=100):
        """Initializes the Wiimote IMU publisher.
    
        Parameters:
            wiiLock: a threading.Lock object that will be used by the accelerator data sender,
                     the gyro data sender, and the Wiimote driver to stay out of each
                     others' hair.
            wiiMote: a bluetooth-connected, calibrated WIIMote instance
            freq:    the message sending frequency in messages/sec. Max is 100, because
                     the Wiimote only samples the sensors at 100Hz.
        """
        
        WiimoteDataSender.__init__(self, wiiLock, wiiMote, freq)
        
        self.pub = rospy.Publisher('imu_data', Imu)        
        
    def run(self):
        """Loop that obtains the latest wiimote state, publishes the IMU data, and sleeps.
        
        The IMU message, if fully filled in, contains information on orientation,
        acceleration (in m/s^2), and angular rate (in radians/sec). For each of
        these quantities, the IMU message format also wants the corresponding
        covariance matrix.
        
        Wiimote only gives us acceleration and angular rate. So we ensure that the orientation
        data entry is marked invalid. We do this by setting the first
        entry of its associated covariance matrix to -1. The covariance
        matrices are the 3x3 matrix with the axes' variance in the 
        diagonal. We obtain the variance from the Wiimote instance.  
        """
        
        while True:
            (canonicalAccel, canonicalAngleRate) = self.obtainWiimoteData()
            
            msg = Imu(header=None,
                      orientation=None,                                       # will default to [0.,0.,0.,0],
                      orientation_covariance=[-1,0.,0.,0.,0.,0.,0.,0.,0],     # -1 indicates that orientation is unknown
                      angular_velocity=None,
                      angular_velocity_covariance=self.angular_velocity_covariance,
                      linear_acceleration=None,
                      linear_acceleration_covariance=self.linear_acceleration_covariance)
                      
            msg.angular_velocity.x = canonicalAngleRate[PHI]
            msg.angular_velocity.y = canonicalAngleRate[THETA]
            msg.angular_velocity.z = canonicalAngleRate[PSI]
            
            msg.linear_acceleration.x = canonicalAccel[X]
            msg.linear_acceleration.y = canonicalAccel[Y]
            msg.linear_acceleration.z = canonicalAccel[Z]
            
            measureTime = self.wiistate.time
            timeSecs = int(measureTime)
            timeNSecs = int(abs(timeSecs - measureTime) * 10**9)
            msg.header.stamp.secs = timeSecs
            msg.header.stamp.nsecs = timeNSecs
            
            self.pub.publish(msg)
            
            rospy.logdebug("IMU accel: " + str(canonicalAccel) + "; IMU angular rate: " + str(canonicalAngleRate))
            rospy.sleep(self.sleepDuration)
            
class JoySender(WiimoteDataSender):
    """Broadcasting Wiimote accelerator and gyro readings as Joy(stick) messages to Topic imu_data"""
    
    def __init__(self, wiiLock, wiiMote, freq=100):
        """Initializes the Wiimote Joy(stick) publisher.
    
        Parameters:
            wiiLock: a threading.Lock object that will be used by the accelerator data sender,
                     the gyro data sender, and the Wiimote driver to stay out of each
                     others' hair.
            wiiMote: a bluetooth-connected, calibrated WIIMote instance
            freq:    the message sending frequency in messages/sec. Max is 100, because
                     the Wiimote only samples the sensors at 100Hz.
        """
        
        WiimoteDataSender.__init__(self, wiiLock, wiiMote, freq)

        
        self.pub = rospy.Publisher('joy', Joy)        
        
    def run(self):
        """Loop that obtains the latest wiimote state, publishes the joystick data, and sleeps.
        
        The Joy.msg message types calls for just two fields: float32[] axes, and int32[] buttons.
        """
        
        while True:
            (canonicalAccel, canonicalAngleRate) = self.obtainWiimoteData()
            
            msg = Joy(# the Joy msg does not have a header :-( header=None,
                      axes = [canonicalAccel[X], canonicalAccel[Y], canonicalAccel[Z],
                              canonicalAngleRate[PHI], canonicalAngleRate[THETA], canonicalAngleRate[PSI]],
                      buttons = None)
                      
            theButtons = []
            theButtons.append(self.wiistate.buttons[BTN_1])
            theButtons.append(self.wiistate.buttons[BTN_2])
            theButtons.append(self.wiistate.buttons[BTN_PLUS])
            theButtons.append(self.wiistate.buttons[BTN_MINUS])
            theButtons.append(self.wiistate.buttons[BTN_A])
            theButtons.append(self.wiistate.buttons[BTN_B])
            theButtons.append(self.wiistate.buttons[BTN_UP])
            theButtons.append(self.wiistate.buttons[BTN_DOWN])
            theButtons.append(self.wiistate.buttons[BTN_LEFT])
            theButtons.append(self.wiistate.buttons[BTN_RIGHT])
            theButtons.append(self.wiistate.buttons[BTN_HOME])
            
            msg.buttons = theButtons
            
            measureTime = self.wiistate.time
            timeSecs = int(measureTime)
            timeNSecs = int(abs(timeSecs - measureTime) * 10**9)
            # the Joy msg does not have a header :-(
            # msg.header.stamp.secs = timeSecs
            # msg.header.stamp.nsecs = timeNSecs
            
            self.pub.publish(msg)
            
            rospy.logdebug("Joy buttons: " + str(theButtons) + "; Joy accel: " + str(canonicalAccel) + "; Joy angular rate: " + str(canonicalAngleRate))
            rospy.sleep(self.sleepDuration)            
            
        
if __name__ == '__main__':
    try:
        runWiimoteNode()
    except Exception, e:
        print(str(e))
