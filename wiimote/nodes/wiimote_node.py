#!/usr/bin/python
################################################################################
#
# File:         wiimode_node.py
# RCS:          $Header: $
# Description:  Top level ROS node that publishes Wiimote data
#               and allows Wiimote rumble/LED setting.
# Author:       Andreas Paepcke
# Created:      Thu Sep 10 10:31:44 2009
# Modified:     Fri Jan 14 10:51:11 2011 (Andreas Paepcke) paepcke@bhb.willowgarage.com
# Language:     Python
# Package:      N/A
# Status:       Experimental (Do Not Distribute)
#
# (c) Copyright 2009, Willow Garage, all rights reserved.
#
################################################################################
#
# Revisions:
#
# Thu Mar 18 10:56:09 2010 (David Lu) davidlu@wustl.edu
#  Enabled nunchuk message publishing
# Fri Oct 29 08:58:21 2010 (Miguel Angel Julian Aguilar, QBO Project)
#    miguel.angel@thecorpora.com
#    Enabled classic controller message publishing
# Mon Nov 08 11:46:58 2010 (David Lu) davidlu@wustl.edu
#  Added calibrated nunchuk information, changed /joy to /wiijoy
#  Only publish on /wiimote/nunchuk if attached
# Tue Jul 05, 2011 (Chad Rockey) chadrockey@gmail.com
#  Removed LED and Rumble Feedback
#  Added support for sensor_msgs/JoyFeedbackArray
################################################################################
#!/usr/bin/env python

"""The wiimote_node broadcasts four topics, and listens to messages that control
the Wiimote stick's rumble (vibration) and LEDs. Transmitted topics (@100Hz):

   o wiijoy            Messages contain the accelerometer and gyro axis data, and all button states.
   o imu/data          Messages contain the accelerometer and gyro axis data, and covariance matrices
   o wiimote/state     the wiijoy and /imu messages, the button states, the LED states,
                       rumble (i.e. vibrator) state, IR light sensor readings, time since last zeroed, 
                       and battery state. See State.msg
   o imu/is_calibrated Latched message
   o nunchuk           Joy messages using the nunchuk as a joystick
   o classic           Joy messages using the nunchuck as a joystic
                 
The node listens to the following messages:

   o joy/set_feedback
		 Topic that listens to sensor_msgs/JoyFeedbackArray messages.  This controls the LEDs and the Rumble.  There are 4 LEDs with ids of 0 through 3.  The "Player 1" LED is id 0; the "Player 4" LED is id3.  The Wiimote only has one rumble, so it is id 0.
   o imu/calibrate
                 Request to calibrate the device.
                 
No command line parameters.
"""

# Code structure: The main thread spawns one thread each for the 
# four message senders, and one thread each for the message listeners.
# The respective classes are IMUSender, JoySender, NunSender and WiiSender for
# topic sending, and WiimoteListeners for the two message listeners.
#
# The Wiimote driver is encapsulated in class WIIMote (see WIIMote.py).
# That class' singleton instance runs in its own thread, and uses 
# the third-party cwiid access software.


# TODO: Removal of gyro is noticed (covar[0,0]<--1). But software does not notice plugging in.
# TODO: Command line option: --no-zeroing

# -------- Python Standard Modules:
import sys
import threading
import traceback
import time

# -------- ROS-Related Modules:
#import roslib; roslib.load_manifest('wiimote') # old fuerte code
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JoyFeedback
from sensor_msgs.msg import JoyFeedbackArray
from wiimote.msg import IrSourceInfo
from wiimote.msg import State

# -------- WIIMote Modules:
from wiimote.wiimoteExceptions import *
from wiimote.wiimoteConstants import *
import wiimote.WIIMote
import wiimote.wiiutils

GATHER_CALIBRATION_STATS = True

class WiimoteNode():
    

    def runWiimoteNode(self):
        """Initialize the wiimote_node, establishing its name for communication with the Master"""
    
        # All exceptions will end up in the __main__ section
        # and are handled there:
        
        rospy.init_node('wiimote', anonymous=True, log_level=rospy.ERROR) # log_level=rospy.DEBUG
        wiimoteDevice = wiimote.WIIMote.WIIMote()
        wiimoteDevice.zeroDevice()
        
        try:
            IMUSender(wiimoteDevice, freq=100).start()
            JoySender(wiimoteDevice, freq=100).start()
            WiiSender(wiimoteDevice, freq=100).start()
            NunSender(wiimoteDevice, freq=100).start()
	    ClasSender(wiimoteDevice, freq=100).start()
            WiimoteListeners(wiimoteDevice).start()
            
            rospy.spin()
        
        except:    
            rospy.loginfo("Error in startup")
	    rospy.loginfo(sys.exc_info()[0])
        finally:
            try:
                wiimoteDevice.setRumble(False)
                wiimoteDevice.setLEDs([False, False, False, False])
                wiimoteDevice.shutdown()
            except:
                pass
    
    def shutdown(self):
        try:
            IMUSender.stop
            JoySender.stop
            WiiSender.stop
            NunSender.stop
            WiimoteListener.stop
        except:
            pass
        
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
        """Scale accelerator, nunchuk accelerator, and gyro readings to be m/sec^2, m/sec^2 and radians/sec, respectively."""
        
        try: 
    	    # Convert acceleration, which is in g's into m/sec^2:
    	    canonicalAccel = self.wiistate.acc.scale(EARTH_GRAVITY)

            # If nunchuk is connected, then 
            # convert nunchuk acceleration into m/sec^2
            if self.wiistate.nunchukPresent:
                canonicalNunchukAccel = self.wiistate.nunchukAcc.scale(EARTH_GRAVITY)
            else:
                canonicalNunchukAccel = None
    	        
    	    # If the gyro is connected, then 
    	    # Convert gyro reading to radians/sec (see wiimoteConstants.py
    	    # for origin of this scale factor):
    	    if self.wiistate.motionPlusPresent:
    	        canonicalAngleRate = self.wiistate.angleRate.scale(GYRO_SCALE_FACTOR)
    	    else:
    	         canonicalAngleRate = None
    	    
    	    return [canonicalAccel, canonicalNunchukAccel, canonicalAngleRate]
        except AttributeError:
            # An attribute error here occurs when user shuts
            # off the Wiimote before stopping the wiimote_node:
            rospy.loginfo(self.threadName + " shutting down.")
            exit(0)

            
class IMUSender(WiimoteDataSender):
    """Broadcasting Wiimote accelerator and gyro readings as IMU messages to Topic sensor_data/Imu"""
    
    def __init__(self, wiiMote, freq=100):
        """Initializes the Wiimote IMU publisher.
    
        Parameters:
            wiiMote: a bluetooth-connected, calibrated WIIMote instance
            freq:    the message sending frequency in messages/sec. Max is 100, because
                     the Wiimote only samples the sensors at 100Hz.
        """
        
        WiimoteDataSender.__init__(self, wiiMote, freq)
        
        self.pub = rospy.Publisher('imu/data', Imu, queue_size=1)
        
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
        
        rospy.loginfo("Wiimote IMU publisher starting (topic /imu/data).")
        self.threadName = "IMU topic Publisher"
        try:
            while not rospy.is_shutdown():
                (canonicalAccel, canonicalNunchukAccel, canonicalAngleRate) = self.obtainWiimoteData()
                
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
                
		try:
		  self.pub.publish(msg)
		except rospy.ROSException:
		  rospy.loginfo("Topic imu/data closed. Shutting down Imu sender.")
		  exit(0)
                
                #rospy.logdebug("IMU state:")
                #rospy.logdebug("    IMU accel: " + str(canonicalAccel) + "\n    IMU angular rate: " + str(canonicalAngleRate))
                rospy.sleep(self.sleepDuration)
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutdown request. Shutting down Imu sender.")
            exit(0)
            
            
class JoySender(WiimoteDataSender):
    
    """Broadcasting Wiimote accelerator and gyro readings as Joy(stick) messages to Topic joy"""
    
    def __init__(self, wiiMote, freq=100):
        """Initializes the Wiimote Joy(stick) publisher.
    
        Parameters:
            wiiMote: a bluetooth-connected, calibrated WIIMote instance
            freq:    the message sending frequency in messages/sec. Max is 100, because
                     the Wiimote only samples the sensors at 100Hz.
        """
        
        WiimoteDataSender.__init__(self, wiiMote, freq)

        
        self.pub = rospy.Publisher('joy', Joy, queue_size=1)
        
    def run(self):
        """Loop that obtains the latest wiimote state, publishes the joystick data, and sleeps.
        
        The Joy.msg message types calls for just two fields: float32[] axes, and int32[] buttons.
        """
        
        rospy.loginfo("Wiimote joystick publisher starting (topic wiijoy).")
        self.threadName = "Joy topic Publisher"
        try:
            while not rospy.is_shutdown():
                (canonicalAccel, canonicalNunchukAccel, canonicalAngleRate) = self.obtainWiimoteData()
                
                msg = Joy(header=None,
                          axes=[canonicalAccel[X], canonicalAccel[Y], canonicalAccel[Z]],
                          buttons=None)
                
                # If a gyro is attached to the Wiimote, we add the
                # gyro information:
                if self.wiistate.motionPlusPresent:
                    msg.axes.extend([canonicalAngleRate[PHI], canonicalAngleRate[THETA], canonicalAngleRate[PSI]])
                          
                # Fill in the ROS message's buttons field (there *must* be
                #     a better way in python to declare an array of 11 zeroes...]

                theButtons = [False,False,False,False,False,False,False,False,False,False,False]
                theButtons[State.MSG_BTN_1]     = self.wiistate.buttons[BTN_1]
                theButtons[State.MSG_BTN_2]     = self.wiistate.buttons[BTN_2]
                theButtons[State.MSG_BTN_A]     = self.wiistate.buttons[BTN_A]
                theButtons[State.MSG_BTN_B]     = self.wiistate.buttons[BTN_B]
                theButtons[State.MSG_BTN_PLUS]  = self.wiistate.buttons[BTN_PLUS]
                theButtons[State.MSG_BTN_MINUS] = self.wiistate.buttons[BTN_MINUS]
                theButtons[State.MSG_BTN_LEFT]  = self.wiistate.buttons[BTN_LEFT]
                theButtons[State.MSG_BTN_RIGHT] = self.wiistate.buttons[BTN_RIGHT]
                theButtons[State.MSG_BTN_UP]    = self.wiistate.buttons[BTN_UP]
                theButtons[State.MSG_BTN_DOWN]  = self.wiistate.buttons[BTN_DOWN]
                theButtons[State.MSG_BTN_HOME]  = self.wiistate.buttons[BTN_HOME]

                msg.buttons = theButtons
                
                measureTime = self.wiistate.time
                timeSecs = int(measureTime)
                timeNSecs = int(abs(timeSecs - measureTime) * 10**9)
                # Add the timestamp
                msg.header.stamp.secs = timeSecs
                msg.header.stamp.nsecs = timeNSecs
                
		try:
		  self.pub.publish(msg)
		except rospy.ROSException:
		  rospy.loginfo("Topic wiijoy closed. Shutting down Joy sender.")
		  exit(0)

                #rospy.logdebug("Joystick state:")
                #rospy.logdebug("    Joy buttons: " + str(theButtons) + "\n    Joy accel: " + str(canonicalAccel) + "\n    Joy angular rate: " + str(canonicalAngleRate))
                rospy.sleep(self.sleepDuration)
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutdown request. Shutting down Joy sender.")
            exit(0)

class NunSender(WiimoteDataSender):
    
    """Broadcasting nunchuk accelerator and joystick readings as Joy(stick) messages to Topic joy"""
    
    def __init__(self, wiiMote, freq=100):
        """Initializes the nunchuk Joy(stick) publisher.
    
        Parameters:
            wiiMote: a bluetooth-connected, calibrated WIIMote instance
            freq:    the message sending frequency in messages/sec. Max is 100, because
                     the Wiimote only samples the sensors at 100Hz.
        """
        
        WiimoteDataSender.__init__(self, wiiMote, freq)

        
        
        # Set 'pub' to none here, and check for none-ness in the
	# loop below so as not to start this publisher unnecessarily.
        self.pub = None
	
    def run(self):
        """Loop that obtains the latest nunchuk state, publishes the joystick data, and sleeps.
        
        The Joy.msg message types calls for just two fields: float32[] axes, and int32[] buttons.
        """
        
        self.threadName = "nunchuk Joy topic Publisher"
        try:
            while not rospy.is_shutdown():
                rospy.sleep(self.sleepDuration)
                (canonicalAccel, scaledAcc, canonicalAngleRate) = self.obtainWiimoteData()
                if not self.wiistate.nunchukPresent:
                    continue
                if self.pub is None:
                    self.pub = rospy.Publisher('/wiimote/nunchuk', Joy, queue_size=1)
                    rospy.loginfo("Wiimote Nunchuk joystick publisher starting (topic nunchuk).")
                
                (joyx, joyy) = self.wiistate.nunchukStick
                                
                msg = Joy(header=None,
                          axes=[joyx, joyy,
                                scaledAcc[X], scaledAcc[Y], scaledAcc[Z]],
                          buttons=None)

                theButtons = [False,False]
                theButtons[State.MSG_BTN_Z]     = self.wiistate.nunchukButtons[BTN_Z]
                theButtons[State.MSG_BTN_C]     = self.wiistate.nunchukButtons[BTN_C]

                msg.buttons = theButtons
                
                measureTime = self.wiistate.time
                timeSecs = int(measureTime)
                timeNSecs = int(abs(timeSecs - measureTime) * 10**9)
                msg.header.stamp.secs = timeSecs
                msg.header.stamp.nsecs = timeNSecs
                
		try:
		  self.pub.publish(msg)
		except rospy.ROSException:
		  rospy.loginfo("Topic /wiimote/nunchuk closed. Shutting down Nun sender.")
		  exit(0)
                
                #rospy.logdebug("nunchuk state:")
                #rospy.logdebug("    nunchuk buttons: " + str(theButtons) + "\n    Nuchuck axes: " + str(msg.axes) + "\n")

        except rospy.ROSInterruptException:
            rospy.loginfo("Shutdown request. Shutting down Nun sender.")
            exit(0)

class ClasSender(WiimoteDataSender):
    
    """Broadcasting Classic Controller joystick readings as Joy(stick) messages to Topic joy"""
    
    def __init__(self, wiiMote, freq=100):
        """Initializes the Classic Controller Joy(stick) publisher.
    
        Parameters:
            wiiMote: a bluetooth-connected, calibrated WIIMote instance
            freq:    the message sending frequency in messages/sec. Max is 100, because
                     the Wiimote only samples the sensors at 100Hz.
        """
        
        WiimoteDataSender.__init__(self, wiiMote, freq)

        # Set 'pub' to none here, and check for none-ness in the
	# loop below so as not to start this publisher unnecessarily.
        self.pub = None
        
    def run(self):
        """Loop that obtains the latest classic controller state, publishes the joystick data, and sleeps.
        
        The Joy.msg message types calls for just two fields: float32[] axes, and int32[] buttons.
        """

	self.threadName = "Classic Controller Joy topic Publisher"
        try:
            while not rospy.is_shutdown():
                rospy.sleep(self.sleepDuration)
                self.obtainWiimoteData()
		
                if not self.wiistate.classicPresent:
                    continue
		if self.pub is None:
		    self.pub = rospy.Publisher('/wiimote/classic', Joy)
		    rospy.loginfo("Wiimote Classic Controller joystick publisher starting (topic /wiimote/classic).")
	  
                (l_joyx, l_joyy) = self.wiistate.classicStickLeft
                (r_joyx, r_joyy) = self.wiistate.classicStickRight
                # scale the joystick to [-1, 1]
                l_joyx = -(l_joyx-33)/27.
                l_joyy = (l_joyy-33)/27.
                r_joyx = -(r_joyx-15)/13.
                r_joyy = (r_joyy-15)/13.
                # create a deadzone in the middle
                if abs(l_joyx) < .05:
                    l_joyx = 0
                if abs(l_joyy) < .05:
                    l_joyy = 0
                if abs(r_joyx) < .05:
                    r_joyx = 0
                if abs(r_joyy) < .05:
                    r_joyy = 0
                
                msg = Joy(header=None,
                          axes=[l_joyx, l_joyy,r_joyx, r_joyy],
                          buttons=None)

                theButtons = [False,False,False,False,False,False,False,False,False,False,False,False,False,False,False]
                theButtons[State.MSG_CLASSIC_BTN_X]     = self.wiistate.classicButtons[CLASSIC_BTN_X]
                theButtons[State.MSG_CLASSIC_BTN_Y]     = self.wiistate.classicButtons[CLASSIC_BTN_Y]
                theButtons[State.MSG_CLASSIC_BTN_A]     = self.wiistate.classicButtons[CLASSIC_BTN_A]
                theButtons[State.MSG_CLASSIC_BTN_B]     = self.wiistate.classicButtons[CLASSIC_BTN_B]
                theButtons[State.MSG_CLASSIC_BTN_PLUS]     = self.wiistate.classicButtons[CLASSIC_BTN_PLUS]
                theButtons[State.MSG_CLASSIC_BTN_MINUS]     = self.wiistate.classicButtons[CLASSIC_BTN_MINUS]
                theButtons[State.MSG_CLASSIC_BTN_LEFT]     = self.wiistate.classicButtons[CLASSIC_BTN_LEFT]
                theButtons[State.MSG_CLASSIC_BTN_RIGHT]     = self.wiistate.classicButtons[CLASSIC_BTN_RIGHT]
                theButtons[State.MSG_CLASSIC_BTN_UP]     = self.wiistate.classicButtons[CLASSIC_BTN_UP]
                theButtons[State.MSG_CLASSIC_BTN_DOWN]     = self.wiistate.classicButtons[CLASSIC_BTN_DOWN]
                theButtons[State.MSG_CLASSIC_BTN_HOME]     = self.wiistate.classicButtons[CLASSIC_BTN_HOME]
                theButtons[State.MSG_CLASSIC_BTN_L]     = self.wiistate.classicButtons[CLASSIC_BTN_L]
                theButtons[State.MSG_CLASSIC_BTN_R]     = self.wiistate.classicButtons[CLASSIC_BTN_R]
                theButtons[State.MSG_CLASSIC_BTN_ZL]     = self.wiistate.classicButtons[CLASSIC_BTN_ZL]
                theButtons[State.MSG_CLASSIC_BTN_ZR]     = self.wiistate.classicButtons[CLASSIC_BTN_ZR]

                msg.buttons = theButtons
                
                measureTime = self.wiistate.time
                timeSecs = int(measureTime)
                timeNSecs = int(abs(timeSecs - measureTime) * 10**9)
                msg.header.stamp.secs = timeSecs
                msg.header.stamp.nsecs = timeNSecs
                
		try:
		  self.pub.publish(msg)
		except rospy.ROSException:
		  rospy.loginfo("Topic /wiimote/classic closed. Shutting down Clas sender.")
		  exit(0)

                #rospy.logdebug("Classic Controller state:")
                #rospy.logdebug("    Classic Controller buttons: " + str(theButtons) + "\n    Classic Controller axes: " + str(msg.axes) + "\n")

        except rospy.ROSInterruptException:
            rospy.loginfo("Shutdown request. Shutting down Clas sender.")
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
        
        self.pub = rospy.Publisher('/wiimote/state', State, queue_size=1)
        
    def run(self):
        """Loop that obtains the latest wiimote state, publishes the data, and sleeps.
        
        The wiimote message, if fully filled in, contains information in common with Imu.msg:
        acceleration (in m/s^2), and angular rate (in radians/sec). For each of
        these quantities, the IMU message format also wants the corresponding
        covariance matrix.
        
        The covariance matrices are the 3x3 matrix with the axes' variance in the 
        diagonal. We obtain the variance from the Wiimote instance.  
        """
        
        rospy.loginfo("Wiimote state publisher starting (topic /wiimote/state).")
        self.threadName = "Wiimote topic Publisher"
        try:
            while not rospy.is_shutdown():
                rospy.sleep(self.sleepDuration)
                (canonicalAccel, canonicalNunchukAccel, canonicalAngleRate) = self.obtainWiimoteData()
                
                zeroingTimeSecs = int(self.wiiMote.lastZeroingTime)
                zeroingTimeNSecs = int((self.wiiMote.lastZeroingTime - zeroingTimeSecs) * 10**9)
                msg = State(header=None,
                            angular_velocity_zeroed=None,
                            angular_velocity_raw=None,
                            angular_velocity_covariance=self.angular_velocity_covariance,
                            linear_acceleration_zeroed=None,
                            linear_acceleration_raw=None,
                            linear_acceleration_covariance=self.linear_acceleration_covariance,
                            nunchuk_acceleration_zeroed=None,
                            nunchuk_acceleration_raw=None,
                            nunchuk_joystick_zeroed=None,
                            nunchuk_joystick_raw=None,
                            buttons=[False,False,False,False,False,False,False,False,False,False],
                            nunchuk_buttons=[False,False],
                            rumble=False,
                            LEDs=None,
                            ir_tracking = None,
                            raw_battery=None,
                            percent_battery=None,
                            zeroing_time=rospy.Time(zeroingTimeSecs, zeroingTimeNSecs),
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

                if self.wiistate.nunchukPresent:
                    msg.nunchuk_acceleration_zeroed.x = canonicalNunchukAccel[X]
                    msg.nunchuk_acceleration_zeroed.y = canonicalNunchukAccel[Y]
                    msg.nunchuk_acceleration_zeroed.z = canonicalNunchukAccel[Z]
                    
                    msg.nunchuk_acceleration_raw.x = self.wiistate.nunchukAccRaw[X]
                    msg.nunchuk_acceleration_raw.y = self.wiistate.nunchukAccRaw[Y]
                    msg.nunchuk_acceleration_raw.z = self.wiistate.nunchukAccRaw[Z]

                    msg.nunchuk_joystick_zeroed = self.wiistate.nunchukStick
                    msg.nunchuk_joystick_raw    = self.wiistate.nunchukStickRaw
                    moreButtons = []
                    moreButtons.append(self.wiistate.nunchukButtons[BTN_Z])
                    moreButtons.append(self.wiistate.nunchukButtons[BTN_C])
                    msg.nunchuk_buttons = moreButtons

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
                for indx in range(len(msg.LEDs)):
                    if ledStates[indx]: 
                        msg.LEDs[indx] = True
                    else:
                        msg.LEDs[indx] = False

                msg.buttons = theButtons

                msg.raw_battery = self.wiiMote.getBattery()
                msg.percent_battery = msg.raw_battery * 100./self.wiiMote.BATTERY_MAX
    
                irSources = self.wiistate.IRSources
                
                for irSensorIndx in range(NUM_IR_SENSORS):
                    if irSources[irSensorIndx] is not None:
                        # Did hardware deliver IR source position for this IR sensor?
                        try:
                          pos  = irSources[irSensorIndx]['pos']
                        except KeyError:
                            # If no position information from this IR sensor, use INVALID for the dimensions:
                            msg.ir_tracking.append(IrSourceInfo(State.INVALID_FLOAT, State.INVALID_FLOAT, State.INVALID))
                        # The following else is unusual: its statements are bypassed is except clause had control:
                        else:
                            # Have IR position info from this IR sensor. We use the IR_source_info
                            # message type. Get size (intensity?):
                            try: 
                                size = irSources[irSensorIndx]['size']
                            except KeyError:
                                # If the driver did not deliver size information, indicate by using INVALID:
                                size = State.INVALID
                            lightInfo = IrSourceInfo(pos[0], pos[1], size)
                            msg.ir_tracking.append(lightInfo)
                    else:
                        msg.ir_tracking.append(IrSourceInfo(State.INVALID_FLOAT, State.INVALID_FLOAT, State.INVALID))

                measureTime = self.wiistate.time
                timeSecs = int(measureTime)
                timeNSecs = int(abs(timeSecs - measureTime) * 10**9)
                msg.header.stamp.secs = timeSecs
                msg.header.stamp.nsecs = timeNSecs
                
		try:
		  self.pub.publish(msg)
		except rospy.ROSException:
		  rospy.loginfo("Topic /wiimote/state closed. Shutting down Wiimote sender.")
		  exit(0)

                #rospy.logdebug("Wiimote state:")
                #rospy.logdebug("    Accel: " + str(canonicalAccel) + "\n    Angular rate: " + str(canonicalAngleRate))
                #rospy.logdebug("    Rumble: " + str(msg.rumble) + "\n    Battery: [" + str(msg.raw_battery) + "," + str(msg.percent_battery))
                #rospy.logdebug("    IR positions: " + str(msg.ir_tracking))
                                

        except rospy.ROSInterruptException:
            rospy.loginfo("Shutdown request. Shutting down Wiimote sender.")
            exit(0)
        
class WiimoteListeners(threading.Thread):
    """Listen for request to rumble and LED blinking.
    """
    
    def __init__(self, wiiMote):
        
        threading.Thread.__init__(self)
        self.wiiMote = wiiMote    
        
        self.ledCommands = [False, False, False, False]
	self.rumbleCommand = False
        
        # Even though this thread mostly listens,
        # we do publish the is_calibrated() message
        # here, because this msg is so closely related
        # to the calibrate() service:
        self.is_calibratedPublisher = rospy.Publisher('/imu/is_calibrated', Bool, latch=True, queue_size=1)
        # We'll always just reuse this msg object:        
        self.is_CalibratedResponseMsg = Bool();

        # Initialize the latched is_calibrated state. We use
	# the result of the initial zeroing, when the services
	# were first started and the the user was asked to
	# push the two buttons for pairing:

        self.is_CalibratedResponseMsg.data = self.wiiMote.latestCalibrationSuccessful;
        self.is_calibratedPublisher.publish(self.is_CalibratedResponseMsg)
        
    def run(self):
        
      def feedbackCallback(msg):
        """The callback for handle the feedback array messages and sending that to the Wiimote"""
        for fb in msg.array:
	  if fb.type == JoyFeedback.TYPE_LED:
	    try:
	      if fb.intensity >= 0.5:
	        self.ledCommands[fb.id] = True
	      else:
		self.ledCommands[fb.id] = False
	    except:
	      rospy.logwarn("LED ID out of bounds, ignoring!")
	  elif fb.type == JoyFeedback.TYPE_RUMBLE:
	    if fb.id == 0:
	      if fb.intensity >= 0.5:
		self.rumbleCommand = True
	      else:
		self.rumbleCommand = False
	    else:
	      rospy.logwarn("RUMBLE ID out of bounds, ignoring!")

	self.wiiMote.setLEDs(self.ledCommands)        
	self.wiiMote.setRumble(self.rumbleCommand)

        
        return


      def calibrateCallback(req):
        """The imu/calibrate service handler."""
          
        rospy.loginfo("Calibration request")
        
        calibrationSuccess = self.wiiMote.zeroDevice()
        
        # Update the latched is_calibrated state:

        self.is_CalibratedResponseMsg.data = calibrationSuccess
        self.is_calibratedPublisher.publish(self.is_CalibratedResponseMsg)
        
        return EmptyResponse()

      # Done with embedded function definitions. Back at the top
      # level of WiimoteListeners' run() function.
       
      # Subscribe to rumble and LED control messages and sit:
      rospy.loginfo("Wiimote feedback listener starting (topic /joy/set_feedback).")
      rospy.Subscriber("joy/set_feedback", JoyFeedbackArray, feedbackCallback)
      rospy.loginfo("Wiimote calibration service starting (topic /imu/calibrate).")
      rospy.Service("imu/calibrate", Empty, calibrateCallback)
      rospy.loginfo("Wiimote latched is_calibrated publisher starting (topic /imu/is_calibrated).")
      
      try:
          rospy.spin()
      except rospy.ROSInterruptException:
        rospy.loginfo("Shutdown request. Shutting down Wiimote listeners.")
        exit(0)
        

if __name__ == '__main__':
    wiimoteNode = WiimoteNode()
    try:
        wiimoteNode.runWiimoteNode()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Shutdown Request.")
    except KeyboardInterrupt, e:
        rospy.loginfo("Received keyboard interrupt.")
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
        if (wiimoteNode is not None):
            wiimoteNode.shutdown()
        rospy.loginfo("Exiting Wiimote node.")
        sys.exit(0)
