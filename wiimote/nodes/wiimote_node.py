#!/usr/bin/python
################################################################################
#
# File:         wiimode_node.py
# RCS:          $Header: $
# Description:  Top level ROS node that publishes Wiimote data
#               and allows Wiimote rumble/LED setting.
# Author:       Andreas Paepcke
# Created:      Thu Sep 10 10:31:44 2009
# Modified:     Thu Sep 24 14:41:10 2009 (Andreas Paepcke) paepcke@anw.willowgarage.com
# Language:     Python
# Package:      N/A
# Status:       Experimental (Do Not Distribute)
#
# (c) Copyright 2009, Willow Garage, all rights reserved.
#
################################################################################

#!/usr/bin/env python

# TODO: Polarity on linear acc y and z gets lost?
# TODO: Removal of gyro is noticed (covar[0,0]<--1). But plugging back in won't rejunenate.
# TODO: Command line option: --no-zeroing
# TODO: Full Wiimote msg type
# TODO: Put ir light positions and sizes into msgs. Search for #****

# -------- Python Standard Modules:
import sys
import threading
import traceback

# -------- ROS-Related Modules:
import roslib; roslib.load_manifest('wiimote')
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from joy.msg import Joy
from wiimote.msg import Wiimote

# -------- WIIMote Modules:
from wiimote.wiimoteExceptions import *
from wiimote.wiimoteConstants import *
import wiimote.WIIMote

def runWiimoteNode():
    """Initialize the wiimote_node, establishing its name for communication with the Master"""

    # All exceptions will end up in the __main__ section
    # and are handled there:
    
    rospy.init_node('wiimote', anonymous=True, log_level=rospy.ERROR) # log_level=rospy.DEBUG
    wiimoteDevice = wiimote.WIIMote.WIIMote()
    wiimoteDevice.zeroDevice()
    IMUSender(wiimoteDevice, freq=100).start()
    JoySender(wiimoteDevice, freq=100).start()
    WiiSender(wiimoteDevice, freq=100).start()
    
    while not rospy.is_shutdown():
        rospy.spin()

class WiimoteDataSender(threading.Thread):
    
    def __init__(self, wiiMote, freq=100):
        
        threading.Thread.__init__(self)
        self.wiiMote = wiiMote
        self.freq = freq
        self.sleepDuration = 1.0 / freq
        
        varianceAccelerator = self.wiiMote.getVarianceAccelerator();
        self.linear_acceleration_covariance = [varianceAccelerator[X], 0., 0.,
                                               0., varianceAccelerator[Y], 0.,
                                               0., 0., varianceAccelerator[Z]]

        varianceGyro = self.wiiMote.getVarianceGyro();
        self.angular_velocity_covariance = [varianceGyro[X], 0., 0.,
                                            0., varianceGyro[Y], 0.,
                                            0., 0., varianceGyro[Z]]
        
        # If no gyro is attached to the Wiimote then we signal
        # the invalidity of angular rate w/ a covariance matrix
        # whose first element is -1:
        self.gyroAbsence_covariance = [-1., 0., 0.,
                                       0., 0., 0.,
                                       0., 0., 0.]
    
    def obtainWiimoteData(self):
        """Retrieve one set of Wiimote measurements from the Wiimote instance. Return scaled accelerator and gyro readings.
        
        We canonicalize both accelerator and gyro data through
        scaling them by constants that turn them into m/sec^2, and 
        radians/sec, respectively.
        
        Return: list of canonicalized accelerator and gyro readings. 
        """
        
        while not rospy.is_shutdown():
            self.wiistate = self.wiiMote.getWiimoteState()
            if self.wiistate is not None and self.wiistate.acc is not None:
                break
            else:
                rospy.sleep(0.1)
            
        return self.canonicalizeWiistate()
        
    def canonicalizeWiistate(self):
        """Scale accelerator and gyro readings to be m/sec^2, and radians/sec, respectively."""
        
        # Convert acceleration, which is in g's into m/sec^@:
        canonicalAccel = self.wiistate.acc.scale(EARTH_GRAVITY)
            
        # If the gyro is connected, then 
        # Convert gyro reading to radians/sec (see wiimoteConstants.py
        # for origin of this scale factor):
        if self.wiistate.motionPlusPresent:
            canonicalAngleRate = self.wiistate.angleRate.scale(GYRO_SCALE_FACTOR)
        else:
             canonicalAngleRate = None
        
        return [canonicalAccel, canonicalAngleRate]


            
class IMUSender(WiimoteDataSender):
    """Broadcasting Wiimote accelerator and gyro readings as IMU messages to Topic imu_data"""
    
    def __init__(self, wiiMote, freq=100):
        """Initializes the Wiimote IMU publisher.
    
        Parameters:
            wiiMote: a bluetooth-connected, calibrated WIIMote instance
            freq:    the message sending frequency in messages/sec. Max is 100, because
                     the Wiimote only samples the sensors at 100Hz.
        """
        
        WiimoteDataSender.__init__(self, wiiMote, freq)
        
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
        
        rospy.loginfo("Wiimote IMU publisher starting (topic /imu_data).")
        try:
            while not rospy.is_shutdown():
                (canonicalAccel, canonicalAngleRate) = self.obtainWiimoteData()
                
                msg = Imu(header=None,
                          orientation=None,                                         # will default to [0.,0.,0.,0],
                          orientation_covariance=[-1.,0.,0.,0.,0.,0.,0.,0.,0.],     # -1 indicates that orientation is unknown
                          angular_velocity=None,
                          angular_velocity_covariance=self.angular_velocity_covariance,
                          linear_acceleration=None,
                          linear_acceleration_covariance=self.linear_acceleration_covariance)
                          
                    
                # If a gyro is plugged into the Wiimote, then note the 
                # angular velocity in the message, else indicate with
                # the special gyroAbsence_covariance matrix that angular
                # velocity is unavailable:      
                if self.wiistate.motionPlusPresent:
                    msg.angular_velocity.x = canonicalAngleRate[PHI]
                    msg.angular_velocity.y = canonicalAngleRate[THETA]
                    msg.angular_velocity.z = canonicalAngleRate[PSI]
                else:
                    msg.angular_velocity_covariance = self.gyroAbsence_covariance
                
                msg.linear_acceleration.x = canonicalAccel[X]
                msg.linear_acceleration.y = canonicalAccel[Y]
                msg.linear_acceleration.z = canonicalAccel[Z]
                
                measureTime = self.wiistate.time
                timeSecs = int(measureTime)
                timeNSecs = int(abs(timeSecs - measureTime) * 10**9)
                msg.header.stamp.secs = timeSecs
                msg.header.stamp.nsecs = timeNSecs
                
                self.pub.publish(msg)
                
                rospy.logdebug("IMU state:")
                rospy.logdebug("    IMU accel: " + str(canonicalAccel) + "\n    IMU angular rate: " + str(canonicalAngleRate))
                rospy.sleep(self.sleepDuration)
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutdown request. Shutting down Imu sender.")
            exit(0)
            
            
class JoySender(WiimoteDataSender):
    """Broadcasting Wiimote accelerator and gyro readings as Joy(stick) messages to Topic imu_data"""
    
    def __init__(self, wiiMote, freq=100):
        """Initializes the Wiimote Joy(stick) publisher.
    
        Parameters:
            wiiMote: a bluetooth-connected, calibrated WIIMote instance
            freq:    the message sending frequency in messages/sec. Max is 100, because
                     the Wiimote only samples the sensors at 100Hz.
        """
        
        WiimoteDataSender.__init__(self, wiiMote, freq)

        
        self.pub = rospy.Publisher('joy', Joy)        
        
    def run(self):
        """Loop that obtains the latest wiimote state, publishes the joystick data, and sleeps.
        
        The Joy.msg message types calls for just two fields: float32[] axes, and int32[] buttons.
        """
        
        rospy.loginfo("Wiimote joystick publisher starting (topic /joy).")
        try:
            while not rospy.is_shutdown():
                (canonicalAccel, canonicalAngleRate) = self.obtainWiimoteData()
                
                msg = Joy(# the Joy msg does not have a header :-( header=None,
                          axes=[canonicalAccel[X], canonicalAccel[Y], canonicalAccel[Z]],
                          buttons=None)
                
                # If a gyro is attached to the Wiimote, we add the
                # gyro information:
                if self.wiistate.motionPlusPresent:
                    msg.axes.extend([canonicalAngleRate[PHI], canonicalAngleRate[THETA], canonicalAngleRate[PSI]])
                          
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
                
                rospy.logdebug("Joystick state:")
                rospy.logdebug("    Joy buttons: " + str(theButtons) + "\n    Joy accel: " + str(canonicalAccel) + "\n    Joy angular rate: " + str(canonicalAngleRate))
                rospy.sleep(self.sleepDuration)
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutdown request. Shutting down Joy sender.")
            exit(0)

class WiiSender(WiimoteDataSender):
    """Broadcasting complete Wiimote messages to Topic wiimote"""
    
    def __init__(self, wiiMote, freq=100):
        """Initializes the full-Wiimote publisher.
    
        Parameters:
            wiiMote: a bluetooth-connected, calibrated WIIMote instance
            freq:    the message sending frequency in messages/sec. Max is 100, because
                     the Wiimote only samples the sensors at 100Hz.
        """
        
        WiimoteDataSender.__init__(self, wiiMote, freq)
        
        self.pub = rospy.Publisher('wiimote', Wiimote)        
        
    def run(self):
        """Loop that obtains the latest wiimote state, publishes the data, and sleeps.
        
        The wiimote message, if fully filled in, contains information in common with imu_data.msg:
        acceleration (in m/s^2), and angular rate (in radians/sec). For each of
        these quantities, the IMU message format also wants the corresponding
        covariance matrix.
        
        The covariance matrices are the 3x3 matrix with the axes' variance in the 
        diagonal. We obtain the variance from the Wiimote instance.  
        """
        
        rospy.loginfo("Wiimote publisher starting (topic /wiimote).")
        try:
            while not rospy.is_shutdown():
                (canonicalAccel, canonicalAngleRate) = self.obtainWiimoteData()
                
                msg = Wiimote(header=None,
                          angular_velocity_zeroed=None,
                          angular_velocity_raw=None,
                          angular_velocity_covariance=self.angular_velocity_covariance,
                          linear_acceleration_zeroed=None,
                          linear_acceleration_raw=None,
                          linear_acceleration_covariance=self.linear_acceleration_covariance,
                          buttons=[0,0,0,0,0,0,0,0,0,0],
                          rumble=0,
                          LEDs=None,
                          ir1_size=-1,
                          ir1_position=None,
                          ir2_size=-1,
                          ir2_position=None,
                          ir3_size=-1,
                          ir3_position=None,
                          ir4_size=-1,
                          ir4_position=None,
                          battery=None,
                          zeroing_time=self.wiiMote.lastZeroingTime,
                          errors=0)
                    
                # If a gyro is plugged into the Wiimote, then note the 
                # angular velocity in the message, else indicate with
                # the special gyroAbsence_covariance matrix that angular
                # velocity is unavailable:      
                if self.wiistate.motionPlusPresent:
                    msg.angular_velocity_zeroed.x = canonicalAngleRate[PHI]
                    msg.angular_velocity_zeroed.y = canonicalAngleRate[THETA]
                    msg.angular_velocity_zeroed.z = canonicalAngleRate[PSI]
                    
                    msg.angular_velocity_raw.x = self.wiistate.angleRateRaw[PHI]
                    msg.angular_velocity_raw.y = self.wiistate.angleRateRaw[THETA]
                    msg.angular_velocity_raw.z = self.wiistate.angleRateRaw[PSI]
                    
                else:
                    msg.angular_velocity_covariance = self.gyroAbsence_covariance
                
                msg.linear_acceleration_zeroed.x = canonicalAccel[X]
                msg.linear_acceleration_zeroed.y = canonicalAccel[Y]
                msg.linear_acceleration_zeroed.z = canonicalAccel[Z]
                
                msg.linear_acceleration_raw.x = self.wiistate.accRaw[X]
                msg.linear_acceleration_raw.y = self.wiistate.accRaw[Y]
                msg.linear_acceleration_raw.z = self.wiistate.accRaw[Z]
                
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
                
                ledStates = self.wiiMote.getLEDs()
                for indx in range(len(msg.LEDs) - 1):
                    if ledStates[indx]: 
                        msg.LEDs[indx] = 1
                    else:
                        msg.LEDs[indx] = 0
                
                msg.buttons = theButtons
    
                batteryRaw = self.wiiMote.getBattery()
                msg.battery[BATTERY_RAW] = batteryRaw
                msg.battery[BATTERY_PERCENTAGE] = batteryRaw * 100./self.wiiMote.BATTERY_MAX
    
                #*****msg.ir1_position = ...
                #*****msg.ir1_size     = ... 
                
                measureTime = self.wiistate.time
                timeSecs = int(measureTime)
                timeNSecs = int(abs(timeSecs - measureTime) * 10**9)
                msg.header.stamp.secs = timeSecs
                msg.header.stamp.nsecs = timeNSecs
                
                self.pub.publish(msg)
                
                rospy.logdebug("Wiimote state:")
                rospy.logdebug("    Accel: " + str(canonicalAccel) + "\n    Angular rate: " + str(canonicalAngleRate))
                rospy.logdebug("    Rumble: " + str(msg.rumble) + "\n    Battery: [" + str(msg.battery[0]) + "," + str(msg.battery[1]))
                rospy.logdebug("    IR1 position: " + str(msg.ir1_position) + " IR1 size: " + str(msg.ir1_position))
                                
                rospy.sleep(self.sleepDuration)
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutdown request. Shutting down Wiimote sender.")
            exit(0)
        
if __name__ == '__main__':
    try:
        runWiimoteNode()
    except rospy.ROSInterruptException:
        rospy.rospy.loginfo("ROS Shutdown Request.")
    except KeyboardInterrupt, e:
        rospy.rospy.loginfo("Received keyboard interrupt.")
    except WiimoteNotFoundError, e:
        rospy.logfatal(str(e))
    except WiimoteEnableError, e:
        rospy.logfatal(str(e))
    except CallbackStackMultInstError, e:
        rospy.logfatal(str(e))
    except CallbackStackEmptyError, e:
        rospy.logfatal(str(e))
    except ResumeNonPausedError, e:
        rospy.logfatal(str(e))
    except CallbackStackEmptyError, e:
        rospy.logfatal(str(e))
    
    except:
        excType, excValue, excTraceback = sys.exc_info()[:3]
        traceback.print_exception(excType, excValue, excTraceback)

    finally:
        rospy.rospy.loginfo("Exiting Wiimote node.")
        sys.exit()
