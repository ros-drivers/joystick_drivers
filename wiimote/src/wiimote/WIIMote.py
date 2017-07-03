#!/usr/bin/env python
################################################################################
#
# File:         WIIMote.py
# RCS:          $Header: $
# Description:  Top Level Wii Remote Control
# Author:       Andreas Paepcke
# Created:      Thu Aug 13 09:00:27 2009
# Modified:     Fri Jan 14 10:48:48 2011 (Andreas Paepcke) paepcke@bhb.willowgarage.com
# Language:     Python
# Package:      N/A
# Status:       Experimental (Do Not Distribute)
#
# 
################################################################################
#
# Revisions:
#
# Fri Jan 14 10:48:11 2011 (Andreas Paepcke) paepcke@bhb.willowgarage.com
#  Added warning to ignore error messages when neither Nunchuk nor WiimotePlus
#  are present.
# Thu Jan 13 17:29:06 2011 (Andreas Paepcke) paepcke@bhb.willowgarage.com
#  Added shutdown exception guard in getRumble()
# Thu Sep 10 10:27:38 2009 (Andreas Paepcke) paepcke@anw.willowgarage.com
#  Added option to lock access to wiiMoteState instance variable.
# Thu Mar 18 10:56:09 2010 (David Lu) davidlu@wustl.edu
#  Enabled nunchuk reports
# Fri Oct 29 08:58:21 2010 (Miguel Angel Julian Aguilar, QBO Project) miguel.angel@thecorpora.com
#  Enabled classic controller reports
# Mon Nov 08 11:44:39 2010 (David Lu) davidlu@wustl.edu
#  Added nunchuk calibration
################################################################################

# ROS-Related Imports

# Python-Internal Imports

import operator
import time
import sys
import threading
from math import *
import tempfile
import os

# Third party modules:

import cwiid
import numpy as np
# ROS modules:

import rospy

# Wiimote modules:

from wiiutils import *
from wiimoteExceptions import *
from wiimoteConstants import *
import wiistate

#;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
#
#    Global Constants
#
#;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

# Note: the Wiimote object in _wm provides a dictionary
#       of some Wiimote state:
#      _wm.state: {'led': 0, 'rpt_mode': 2, 'ext_type': 4, 'buttons': 0, 'rumble': 0, 'error': 0, 'battery': 85}
#;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
#
#    Class WIIMote
#
#;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


class WIIMote(object):
  """Main class for Wiimote device interaction.
  
  This class should be a singleton, or it should have
  only class members/methods.
  
  Public Data attributes:
      wiiMoteState   WIIState object that holds the latest sampled state
      sampleRate     Control Wiimote state samples to take per second
      meanAcc        Triplet with mean of accelerator at rest
      stdevAcc       Triplet with standard deviation of accelerator at rest
      meanGyro       Triplet with mean of gyro (angular rate) at rest
      stdevGyro      Triplet with standard deviation gyro (angular rate) at rest
      
  Public Methods:
      
  
  """

  # Public constants:
  
  BATTERY_MAX = cwiid.BATTERY_MAX  # 208 a.k.a. 0xD0
  
  # Public vars:

  
  # Private constants:

  
  # Private vars:

  _wm = None                 # WIIMote object
  _wiiCallbackStack = None   # Stack for directing Wii driver callbacks

  _startTime = None          # Used for state sampling
  _accList = None            # Collecting accelerator readings for zeroing and others
  _gyroList = None
  _readingsCnt = None        # For counting how many readings were taken
  _accTotal = None           # Summed up acc readings in one AccReading instance
  _gyroTotal = None          # Summed up gyro readings in one AccReading instance
  

  _accNormal = None          # Readings of accelerometer at rest
  _gyroNormal = None         # Readings of gyro at rest

  _nunchukJoyOrig = None     # Initial Reading of the nunchuk's joystick
  
  _LEDMasksOn = [LED1_ON, LED2_ON, LED3_ON, LED4_ON] # OR to turn on
  _LEDMasksOff = [0 | LED2_ON | LED3_ON | LED4_ON, # AND to turn off
                  0 | LED1_ON | LED3_ON | LED4_ON,
                  0 | LED1_ON | LED2_ON | LED4_ON,
                  0 | LED1_ON | LED2_ON | LED3_ON]


  #----------------------------------------
  # __init__
  #------------------

  def __init__(self, theSampleRate=0, wiiStateLock=None, gatherCalibrationStats=False):
    """Instantiate a Wiimote driver instance, which controls one physical Wiimote device.
    
    Parameters:
        theSampleRate: How often to update the instance's wiiMoteState variable:
            theSampleRate= -1: never
            theSampleRate=  0: as often as possible
            theSampleRate=  x: every x seconds   
    """

    self.lastZeroingTime = 0.
    
    self.gatherCalibrationStats = gatherCalibrationStats
    if (self.gatherCalibrationStats):
        self.calibrationSamples = []
        for i in range(NUM_ZEROING_READINGS):
            self.calibrationSamples.append(CalibrationMeasurements())
        
    # Create a threading.Lock instance.
    # The instance variable wiiMoteState is only updated after acquiring that
    # lock. This is true for both reading and writing. The same is
    # true for accesses to: meanAcc, stdevAcc, varAcc, stdevGyro, and varGyro
    # All such accesses happen w/in this class, b/c we have accessors for
    # these variables. The lock is used also in method zeroDevice() as 
    # well, to prevent other threads from retrieving bad values between
    # the time zeroDevice() begins and ends:
    
    self.wiiStateLock = threading.Lock()
    self.wiiMoteState = None        # Object holding a snapshot of the Wiimote state
    self.sampleRate = -1            # How often to update wiiMoteState
                               #    -1: Never
                               #     0: Everytime the underlying system offers state
                               #  else: (Possibly fractional) seconds between updates
  
    # Mean x/y/z of most recent accelerometer zeroing in Gs and metric:
    self.meanAcc = np.array([None, None, None],dtype=np.float64)   
    self.meanAccMetric = np.array([None, None, None],dtype=np.float64)
    # Stdev x/y/z of most recent accelerometer zeroing in Gs and metric:
    self.stdevAcc = np.array([None, None, None],dtype=np.float64)        
    self.stdevAccMetric = np.array([None, None, None],dtype=np.float64)
    # Variance x/y/z of most recent accelerometer zeroing                    
    self.varAcc = np.array([None, None, None],dtype=np.float64)
    
    # Mean x/y/z of most recent gyro zeroing in Gs and metric:
    self.meanGyro = np.array([None, None, None],dtype=np.float64)
    self.meanGyroMetric = np.array([None, None, None],dtype=np.float64)
    # Stdev x/y/z of most recent gyro zeroing in Gs and metric:
    self.stdevGyro = np.array([None, None, None],dtype=np.float64)
    self.stdevGyroMetric = np.array([None, None, None],dtype=np.float64)
    # Variance x/y/z of most recent gyro zeroing                                 
    self.varGyroMetric = np.array([None, None, None],dtype=np.float64)
                                 
    self.latestCalibrationSuccessful = False;
    
    promptUsr("Press buttons 1 and 2 together to pair (within 6 seconds).\n    (If no blinking lights, press power button for ~3 seconds.)")

    try:
      self._wm = cwiid.Wiimote()
    except RuntimeError:
      raise WiimoteNotFoundError("No Wiimote found to pair with.")
      exit()

    rospy.loginfo("Pairing successful.")

    try:
      self._wm.enable(cwiid.FLAG_MOTIONPLUS)
    except RuntimeError:
      raise WiimoteEnableError("Found Wiimote, but could not enable it.")
      exit
      
    self.sampleRate = theSampleRate
    self._startTime = getTimeStamp();

    self._wiiCallbackStack = _WiiCallbackStack(self._wm)

    # Enable reports from the WII:
    self._wm.rpt_mode = cwiid.RPT_ACC | cwiid.RPT_MOTIONPLUS | cwiid.RPT_BTN | cwiid.RPT_IR | cwiid.RPT_NUNCHUK | cwiid.RPT_CLASSIC
    
    # Set accelerometer calibration to factory defaults:
    (factoryZero, factoryOne) = self.getAccFactoryCalibrationSettings()
    self.setAccelerometerCalibration(factoryZero, factoryOne)
    
    # Initialize Gyro zeroing to do nothing:
    self.setGyroCalibration([0,0,0])

    # Set nunchuk calibration to factory defaults.
    if (self._wm.state['ext_type'] == cwiid.EXT_NUNCHUK):
      try:
        (factoryZero, factoryOne) = self.getNunchukFactoryCalibrationSettings()
        self.setNunchukAccelerometerCalibration(factoryZero, factoryOne)
      except:
        pass

    time.sleep(0.2)
    self._wiiCallbackStack.push(self._steadyStateCallback)

    rospy.loginfo("Wiimote activated.")


  #----------------------------------------
  # steadyStateCallback
  #------------------

  def _steadyStateCallback(self, state, theTime):
    #print state
    now = getTimeStamp()
    if now - self._startTime >= self.sampleRate:
        # If this Wiimote driver is to synchronize write
        # access to the wii state variable (which is read from
        # outside), then acquire the lock that was provided
        # by the instantiator of this instance:
        if self.wiiStateLock is not None:
            self.wiiStateLock.acquire()
	try:
	  self.wiiMoteState = wiistate.WIIState(state, theTime, self.getRumble(), self._wm.state['buttons']);
	except ValueError:
	  # A 'Wiimote is closed' error can occur as a race condition
	  # as threads close down after a Cnt-C. Catch those and
	  # ignore:
	  pass
        if self.wiiStateLock is not None:
            self.wiiStateLock.release()
        self._startTime = now

  #----------------------------------------
  # _calibrationCallback
  #---------------------

  def _calibrationCallback(self, state, theTime):
    """Wii's callback destination while zeroing the device."""

    self._warmupCnt += 1
    if self._warmupCnt < NUM_WARMUP_READINGS:
        return

    if self._readingsCnt >= NUM_ZEROING_READINGS:
        return

    thisState = wiistate.WIIState(state, theTime, self.getRumble(), self._wm.state['buttons'])

    # Pull out the accelerometer x,y,z, accumulate in a list:
    self._accList.append(thisState.accRaw)
    
    # Pull out the gyro x,y,z, and build a GyroReading from them.
    # For a few cycles, the Wiimote does not deliver gyro info.
    # When it doesn't, we get a 'None' is unsubscriptable. Ignore 
    # those initial instabilities:
    try:
        self._gyroList.append(thisState.angleRate)
    except TypeError:
        pass
    self._readingsCnt += 1

    if thisState.nunchukPresent and self._nunchukJoyOrig is None:
        self._nunchukJoyOrig = thisState.nunchukStickRaw
        wiistate.WIIState.setNunchukJoystickCalibration(self._nunchukJoyOrig)

    return

  #----------------------------------------
  # zero
  #------------------

  def zeroDevice(self):
    """Find the at-rest values of the accelerometer and the gyro.

    Collect NUM_ZEROING_READINGS readings of acc and gyro. Average them.
    If the standard deviation of any of the six axes exceeds a threshold
    that was determined empirically, then the calibration fails. Else
    the gyro is biased to compensate for its at-rest offset. The offset
    is the abs(mean(Gyro)).
    
    The stdev thresholds are documented in wiimoteConstants.py.
    
    Note that we always use the Wiimote's factory-installed zeroing data.
    In the code below we nonetheless compute the stats for the 
    accelerometer, in case this behavior is to change in the future.
    
    We sleep while the samples are taken. In order to prevent other
    threads from reading bad values for mean/stdev, and variance, 
    we lock access to those vars.
    """

    self._accList = []    # Calibration callback will put samples here (WIIReading()s)
    self._gyroList = []   # Calibration callback will put samples here (WIIReading()s)
    self._readingsCnt = 0
    self._warmupCnt = 0
    # The factory calibration setting for the accelerometer (two values in a tuple):
    accCalibrationOrig = self.getAccelerometerCalibration()
    gyroCalibrationOrig = self.getGyroCalibration()
    accArrays = []        # Place to put raw reading triplets
    gyroArrays = []       # Place to put raw reading triplets
    
    try:
        # Get the samples for accelerometer and gyro:
        self.wiiStateLock.acquire()

        self._wiiCallbackStack.push(self._calibrationCallback)
        
        # Wipe out previous calibration correction data
        # while we gather raw samples:
        wiistate.WIIState.setGyroCalibration([0,0,0])
        wiistate.WIIState.setAccelerometerCalibration([0,0,0], [0,0,0])
                                                              
        while (self._readingsCnt < NUM_ZEROING_READINGS) or (self._warmupCnt < NUM_WARMUP_READINGS):
          time.sleep(.1)
         
        self._wiiCallbackStack.pause()
    finally:
        # Restore the callback that was in force before zeroing:
        self._wiiCallbackStack.pop()
        self.wiiStateLock.release()

    # Compute and store basic statistics about the readings:
    self.computeAccStatistics()
    self.computeGyroStatistics()
    
    # Extract the accelerometer reading triplets from the list of WIIReading()s:
    for accWiiReading in self._accList:
        if accWiiReading is not None:
            oneAccReading = accWiiReading.tuple()
            accArrays.append(oneAccReading)
    accArrays = np.reshape(accArrays, (-1,3))
    
    # Extract the gyro reading triplets from the list of WIIReading()s:
    if (self.motionPlusPresent()):     
    	for gyroReading in self._gyroList:
    	    if (gyroReading is not None):
    	        oneGyroReading = gyroReading.tuple()
    	        gyroArrays.append(oneGyroReading)
    
    if (self.motionPlusPresent()):
        gyroArrays = np.reshape(gyroArrays, (-1,3))
        
        # We now have:
        # [[accX1, accZ1, accZ1]
        #  [accX2, accZ2, accZ2]
        #      ...
        #  ]
        # 
        # and:
        # 
        # [[gyroX1, gyroZ1, gyroZ1]
        #  [gyroX2, gyroZ2, gyroZ2]
        #      ...
        #  ]
        # 
        # Combine all into:
        # 
        # [[accX1, accZ1, accZ1, gyroX1, gyroZ1, gyroZ1]
        #  [accX2, accZ2, accZ2, gyroX2, gyroZ2, gyroZ2]
        #      ...
        #  ]
        # 
       
        allData = np.append(accArrays, gyroArrays, axis=1)
        # Will compare both, accelerometer x/y/z, and gyro x/y/z
        # to their stdev threshold to validate calibration:
        thresholdsArray = THRESHOLDS_ARRAY
    else:
        allData = accArrays
        # Will compare only accelerometer x/y/z to their stdev
        # threshold to validate calibration. No Wiimote+ was
        # detected:
        thresholdsArray = THRESHOLDS_ARRAY[0:3]
      
    # And take the std deviations column-wise:
    stdev = np.std(allData, axis=0)
    
    # See whether any of the six stdevs exceeds the
    # calibration threshold:
    
    isBadCalibration = (stdev > thresholdsArray).any()

    # We always use the factory-installed calibration info,
    self.setAccelerometerCalibration(accCalibrationOrig[0], accCalibrationOrig[1])
        
    if (isBadCalibration):
        self.latestCalibrationSuccessful = False;
        # We can calibrate the Wiimote anyway, if the preference
        # constant in wiimoteConstants.py is set accordingly:
        if (CALIBRATE_WITH_FAILED_CALIBRATION_DATA and self.motionPlusPresent()):
            rospy.loginfo("Failed calibration; using questionable calibration anyway.")
            wiistate.WIIState.setGyroCalibration(self.meanGyro)
        else:
            if (gyroCalibrationOrig is not None):
                rospy.loginfo("Failed calibration; retaining previous calibration.")
                if (self.motionPlusPresent()):
                    wiistate.WIIState.setGyroCalibration(gyroCalibrationOrig)
            else:
                rospy.loginfo("Failed calibration; running without calibration now.")
        return False
    
    # Tell the WIIState factory that all WIIMote state instance creations
    # should correct accelerometer readings automatically, using the 
    # Nintendo-factory-set values: 

    
    # Do WIIState's gyro zero reading, so that future
    # readings can be corrected when a WIIState is created:
    wiistate.WIIState.setGyroCalibration(self.meanGyro)
            
    self.lastZeroingTime = getTimeStamp()
    rospy.loginfo("Calibration successful.")
    self.latestCalibrationSuccessful = True;
    return True;

  #----------------------------------------
  # getWiimoteState
  #------------------

  def getWiimoteState(self):
      """Returns the most recent Wiistate instance. Provides proper locking."""
      
      return self._getInstanceVarCriticalSection("wiimoteState")
  
  #----------------------------------------
  # getMeanAccelerator
  #------------------

  def getMeanAccelerator(self):
      """Accessor that provides locking."""
      
      return self._getInstanceVarCriticalSection("meanAcc")
  
  #----------------------------------------
  # getStdevAccelerator
  #------------------

  def getStdevAccelerator(self):
      """Accessor that provides locking."""
      
      return self._getInstanceVarCriticalSection("stdevAcc")
 
  #----------------------------------------
  # getVarianceAccelerator
  #------------------

  def getVarianceAccelerator(self):
      """Accessor that provides locking."""
      
      return self._getInstanceVarCriticalSection("varAcc")

  #----------------------------------------
  # getMeanGyro
  #------------------

  def getMeanGyro(self):
      """Accessor that provides locking."""
      
      return self._getInstanceVarCriticalSection("meanGyro")
  
  #----------------------------------------
  # getStdevGyro
  #------------------

  def getStdevGyro(self):
      """Accessor that provides locking."""
      
      return self._getInstanceVarCriticalSection("stdevGyro")
 
  #----------------------------------------
  # getVarianceGyro
  #------------------

  def getVarianceGyro(self):
      """Accessor that provides locking."""
      
      return self._getInstanceVarCriticalSection("varGyro")

 
  #----------------------------------------
  # _getInstanceVarCriticalSection
  #------------------

  def _getInstanceVarCriticalSection(self, varName):
      """Return the value of the given instance variable, providing locking service."""
      
      try: 
          self.wiiStateLock.acquire()
          
          if varName == "wiimoteState":
              res = self.wiiMoteState
          elif varName == "meanAcc":
              res = self.meanAcc
          elif varName == "stdevAcc":
              res = self.stdevAcc
          elif varName == "varAcc":
              res = self.varAcc
          elif varName == "meanGyro":
              res = self.meanGyro
          elif varName == "stdevGyro":
              res = self.stdevGyro
          elif varName == "varGyro":
              res = self.varGyroMetric
          else:
              raise ValueError("Instance variable name " + str(varName) + "is not under lock control." )
          
      finally:
          self.wiiStateLock.release()
          return res
 
  #----------------------------------------
  # setRumble
  #------------------

  def setRumble(self, switchPos):
    """Start of stop rumble (i.e. vibration). 1: start; 0: stop""" 
    self._wm.rumble = switchPos


  #----------------------------------------
  # getRumble
  #------------------

  def getRumble(self):
    # Protect against reading exception from reading
    # from an already closed device during shutdown:
    try:
      return self._wm.state['rumble']
    except ValueError:
      pass

  #----------------------------------------
  # setLEDs
  #------------------

  def setLEDs(self, statusList):
    """Set the four Wii LEDs according to statusList

    statusList must be a 4-tuple. Each entry
    is either True/1, False/0, or None. True (or 1) 
    will turn the respective LED on; False (or 0) 
    turns it off, and None leaves the state unchanged.

    """

    currLEDs = self.getLEDs(asInt=True)
    # Cycle through each LED:
    for LED in range(len(statusList)):
      # Should this LED be on?
      if statusList[LED]:
        currLEDs = currLEDs | self._LEDMasksOn[LED]
      # Is this LED to be OFF? (if not, leave it alone)
      elif statusList[LED] is not None:
        currLEDs = currLEDs & self._LEDMasksOff[LED]
    self._wm.led = currLEDs
        

  #----------------------------------------
  # getLEDs
  #------------------

  def getLEDs(self, asInt=False):
    """Get the status of the four Wii LEDs.

    Return value depends on the asInt parameter:
    if asInt=False, the method returns a 4-tuple. 
      Each entry is either True or False. True indicates
      that the respective LED is on; False means off.
    If asInt=True, return value is a bit vector
      indicating which LEDs are on.

    """

    LEDs = self._wm.state['led']
    if asInt:
      return LEDs
    res = []
    if LEDs & LED1_ON:
      res.append(True)
    else:
      res.append(False)

    if LEDs & LED2_ON:
      res.append(True)
    else:
      res.append(False)

    if LEDs & LED3_ON:
      res.append(True)
    else:
      res.append(False)

    if LEDs & LED4_ON:
      res.append(True)
    else:
      res.append(False)

    return res

  #----------------------------------------
  # getBattery
  #------------------

  def getBattery(self):
    """Obtain battery state from Wiimote.

    Maximum charge is BATTERY_MAX.
    """

    return self._wm.state['battery']

  #----------------------------------------
  # getAccelerometerCalibration
  #----------
  
  def getAccelerometerCalibration(self):
      """Returns currently operative accelerometer calibration.
      
      Return value: tuple with calibration for zero reading, and
      calibration or a '1' reading.
     """
      return wiistate.WIIState.getAccelerometerCalibration()
  
  #----------------------------------------
  # getAccFactoryCalibrationSettings
  #------------------

  def getAccFactoryCalibrationSettings(self):
    """Obtain calibration data from accelerometer.

    Retrieve factory-installed calibration data for
    the Wiimote's accelerometer. Returns a two-tuple
    with the calibration numbers for zero and one:

    """

    # Parameter is the Wiimote extension from which
    # the calibration is to be retrieved. 

    factoryCalNums = self._wm.get_acc_cal(cwiid.EXT_NONE);

    return (factoryCalNums[0], factoryCalNums[1])

  #----------------------------------------
  # getNunchukFactoryCalibrationSettings
  #------------------

  def getNunchukFactoryCalibrationSettings(self):
    """Obtain calibration data from nunchuk accelerometer.

    Retrieve factory-installed calibration data for
    the Nunchuk's accelerometer. Returns a two-tuple
    with the calibration numbers for zero and one:

    """
    factoryCalNums = self._wm.get_acc_cal(cwiid.EXT_NUNCHUK);
    return (factoryCalNums[0], factoryCalNums[1])
    
  #----------------------------------------
  # setAccelerometerCalibration
  #----------
  
  def setAccelerometerCalibration(self, zeroReadingList, oneReadingList):
      wiistate.WIIState.setAccelerometerCalibration(np.array(zeroReadingList), np.array(oneReadingList))
    
  def setAccelerometerCalibration(self, zeroReadingNPArray, oneReadingNPArray):
      wiistate.WIIState.setAccelerometerCalibration(zeroReadingNPArray, oneReadingNPArray)

  #----------------------------------------
  # getGyroCalibration
  #------------------

  def getGyroCalibration(self):
      """Return current Gyro zeroing offsets as list x/y/z."""
      return wiistate.WIIState.getGyroCalibration()
  
  #----------------------------------------
  # setGyroCalibration
  #------------------

  def setGyroCalibration(self, gyroTriplet):
      wiistate.WIIState.setGyroCalibration(gyroTriplet)

  #----------------------------------------
  # setNunchukAccelerometerCalibration
  #----------
  
  def setNunchukAccelerometerCalibration(self, zeroReadingList, oneReadingList):
      wiistate.WIIState.setNunchukAccelerometerCalibration(np.array(zeroReadingList), np.array(oneReadingList))
    
  #----------------------------------------
  # motionPlusPresent
  #------------------

  def motionPlusPresent(self):
      """Return True/False to indicate whether a Wiimotion Plus is detected.
      
      Note: The return value is accurate only after at least one 
      Wiimote state has been read. This means that either 
      _steadyStateCallback or _calibrationCallback must have
      run at least once.
      """
      if (self.wiiMoteState is not None):
          return self.wiiMoteState.motionPlusPresent
      else:
          return False

  #----------------------------------------
  # nunchukPresent
  #------------------

  def nunchukPresent(self):
      """Return True/False to indicate whether a Nunchuk is detected.
      
      Note: The return value is accurate only after at least one 
      Wiimote state has been read. This means that either 
      _steadyStateCallback or _calibrationCallback must have
      run at least once.
      """
      if (self.wiiMoteState is not None):
          return self.wiiMoteState.nunchukPresent
      else:
          return False
    
  #----------------------------------------
  # computeAccStatistics
  #------------------
 
  def computeAccStatistics(self):
      """Compute mean and stdev for accelerometer data list self._accList in both Gs and metric m/sec^2"""

      accArrays = []
      self.maxAccReading = np.array([0,0,0], dtype=None, copy=1, order=None, subok=0, ndmin=0)
      for accWiiReading in self._accList:
          if accWiiReading is not None:
              oneAccReading = accWiiReading.tuple()
              accArrays.append(oneAccReading)
              self.maxAccReading = np.maximum(self.maxAccReading, np.abs(oneAccReading))
        
      # Turn list of numpy triplets into three columns containing
      # all x, all y, and all z values, respectively:
      #  [array(10,20,30), array(100,200,300)] ==> [[10   20  30],
      #                                             [100 200 300]]
      # and take the means of each column. We will end up
      # with: [55.0 110.0 165.0]
      
      self.meanAcc = np.vstack(accArrays).mean(axis=0)
      self.meanAccMetric = self.meanAcc * EARTH_GRAVITY
      self.stdevAcc = np.vstack(accArrays).std(axis=0)
      self.stdevAccMetric = self.stdevAcc * EARTH_GRAVITY
      self.varAcc = np.square(self.stdevAccMetric)


  #----------------------------------------
  # computeGyroStatistics
  #------------------
  
  def computeGyroStatistics(self):
      """Compute mean and stdev for gyro data list self._gyroList in both Gs and metric m/sec^2"""      
      gyroArrays = []
      self.maxGyroReading = np.array([0,0,0],dtype=np.float64)        
      for gyroReading in self._gyroList:
          if (gyroReading is not None):
              oneGyroReading = gyroReading.tuple()
              gyroArrays.append(oneGyroReading)
              self.maxGyroReading = np.maximum(self.maxGyroReading, np.abs(oneGyroReading))
            
      if len(gyroArrays) != 0:
          self.meanGyro = np.vstack(gyroArrays).mean(axis=0)
          # Convert to radians/sec:
          self.meanGyroMetric = self.meanGyro * GYRO_SCALE_FACTOR
          self.stdevGyro = np.vstack(gyroArrays).std(axis=0)
          # Convert stdev to radians/sec:
          self.stdevGyroMetric = self.stdevGyro * GYRO_SCALE_FACTOR
          self.varGyroMetric = np.square(self.stdevGyroMetric)

    
  #----------------------------------------
  # printState
  #------------------

  def printState(self):
      log(self.wiiMoteState)
    
         
  #----------------------------------------
  # shutdown
  #------------------

  def shutdown(self):
    self._wm.close()

#;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
#
#    Class WiiCallbackStack
#
#;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


class _WiiCallbackStack(object):
  """Class organizes installation and removal/restoration
  of callback functions for the Wii driver to use. 
  Only one instance of this class is allowed. Additional
  instantiations generate a CallbackStackMultInstError.

  A stack discipline is imposed. Operations:

     - push(<callBackFunc>)        # New function becomes the active
                                   # callback immediately
     - pop() -> <oldCallBackFunc>  # If another function is still on
                                       # the stack, it immediately becomes
                                       # the active callback. If callback
                                     # is paused, resume() is forced.
     - pause()                  # Callbacks are temporarily turned off
     - paused() -> True/False
     - resume(sloppy=True)      # If sloppy=False, resuming when
                                     # callbacks are not paused throws an
                                     # exception.  If sloppy=True, the call is
                                     # a no-op

  """

  _functionStack = []
  _singletonInstance = None  # No instance exists yet.
  _paused = False

  _wm = None                 # The Wii remote driver instance


  #----------------------------------------
  # __init__
  #------------------

  def __init__(self, wiiDriver, sloppy=True):

    if self._singletonInstance:
      if not sloppy:
        raise CallbackStackMultInstError("Can only instantiate one Wii callback stack.")

    self._singletonInstance = self
    self._wm = wiiDriver

  #----------------------------------------
  # push
  #------------------

  def push(self, func):
    """Given function becomes the new WIImote callback function, shadowing 
    the function that is currently on the stack
    """
    
    self._functionStack.append(func)
    self.setcallback(func)

  #----------------------------------------
  # pop
  #------------------

  def pop(self):
    """Wiimote callback function is popped off the stack. New top of stack 
    becomes the new callback function. Old function is returned.
    """
    
    if not self._functionStack:
      raise CallbackStackEmptyError("Attempt to pop empty callback stack")
    _paused = False
    func = self._functionStack.pop()
    self.setcallback(self._functionStack[-1])
    return func

  #----------------------------------------
  # pause
  #------------------

  def pause(self):
    """WIIMote callbacks are temporarily stopped."""
    
    self._wm.disable(cwiid.FLAG_MESG_IFC)    
    self._paused = True
    
  #----------------------------------------
  # resume
  #------------------

  def resume(self, sloppy=True):
    """Resume the (presumably) previously paused WIIMote callback functions.
    If sloppy is True, this method won't complain if pause was not 
    called earlier. If sloppy is False, an exception is raised in 
    that case.
    """
     
    if not self._paused:
      if sloppy:
        return
      else:
        raise ResumeNonPausedError("Can't resume without first pausing.")
    
    if not self._functionStack:
      raise CallbackStackEmptyError("Attempt to pop empty callback stack")

    self._wiiCallbackStack(_functionStack.index[-1])


  #----------------------------------------
  # setcallback
  #------------------

  def setcallback(self, f):
    """Tell WIIMote which function to call when reporting status."""
    
    self._wm.mesg_callback = f
    self._wm.enable(cwiid.FLAG_MESG_IFC)

      
class CalibrationMeasurements():
    
    def __init__(self):
                 # runNum, meanAcc, maxAcc, stdevAcc, meanGyro, maxGyro, stdevGyro,
                 # accVal, devAccVal, stdevFractionAccVal, isOutlierAcc,
                 # gyroVal, devGyroVal, stdevFractionGyroVal, isOutlierGyro):

      pass
             
    def setAccData(self, accArray):
        self.accVal = accArray
    
    def setStdevAcc(self, stdevArray):
        self.stdevAcc = stdevArray
        
    def setMeanAcc(self, meanArray):
        self.meanAcc = meanArray
        
    def setMaxAcc(self, maxArray):
        self.maxAcc = maxArray


    def setGyroData(self, gyroArray):
        self.gyroVal = gyroArray
        
    def setStdevGyro(self, stdevArray):
        self.stdevGyro = stdevArray
        
    def setMeanGyro(self, meanArray):
        self.meanGyro = meanArray

    def setMaxGyro(self, maxArray):
        self.maxGyro = maxArray
      
    def setGyroData(self, gyroVal):
      self.gyroVal = gyroVal
                 
