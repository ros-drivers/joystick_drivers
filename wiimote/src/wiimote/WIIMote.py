#! /usr/bin/python
################################################################################
#
# File:         WIIMote.py
# RCS:          $Header: $
# Description:  Top Level Wii Remote Control
# Author:       Andreas Paepcke
# Created:      Thu Aug 13 09:00:27 2009
# Modified:     Thu Aug 20 09:04:35 2009 (Andreas Paepcke) paepcke@anw.willowgarage.com
# Language:     Python
# Package:      N/A
# Status:       Experimental (Do Not Distribute)
#
# 
################################################################################

import operator
import time
import sys
from math import *

# Third party modules:

import cwiid
import numpy as np

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
  """Main class for Wiimote.
  
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

  # Public vars:

  wiiMoteState = None        # Object holding a snapshot of the Wiimote state
  sampleRate = -1            # How often to update wiiMoteState
                             #    -1: Never
                             #     0: Everytime the underlying system offers state
                             #  else: (Possibly fractional) seconds between updates

   
  meanAcc = [None, None, None] # Mean x/y/z of most recent accelerometer zeroing
                                # Elements of the list are AccReading instances
  stdevAcc = [None, None, None] # Stdev x/y/z of most recent accelerometer zeroing
                                # Elements of the list are AccReading instances

  meanGyro = [None, None, None] # Mean x/y/z of most recent accelerometer zeroing
                                 # Elements of the list are GyroReading instances
  stdevGyro = [None, None, None] # Stdev x/y/z of most recent accelerometer zeroing
                                 # Elements of the list are GyroReading instances

  # Private constants:

  _NUM_ZEROING_READINGS = 10 # Number of readings to take for zeroing acc and gyro

  # Private vars:

  _wm = None                 # WIIMote object
  _wiiCallbackStack = None   # Stack for directing Wii driver callbacks

  _startTime = None          # Used for state sampling
  _accList = None          # Collecting accelerator readings for zeroing and others
  _gyroList = None
  _readingsCnt = None        # For counting how many readings were taken
  _accTotal = None          # Summed up acc readings in one AccReading instance
  _gyroTotal = None          # Summed up gyro readings in one AccReading instance
  

  _accNormal = None         # Readings of accelerometer at rest
  _gyroNormal = None         # Readings of gyro at rest
  
  _LEDMasksOn = [LED1_ON, LED2_ON, LED3_ON, LED4_ON] # OR to turn on
  _LEDMasksOff = [0 | LED2_ON | LED3_ON | LED4_ON, # AND to turn off
                  0 | LED1_ON | LED3_ON | LED4_ON,
                  0 | LED1_ON | LED2_ON | LED4_ON,
                  0 | LED1_ON | LED2_ON | LED3_ON]


  #----------------------------------------
  # __init__
  #------------------

  def __init__(self, theSampleRate=0):
    """Optional sample rate: how often wiiMoteState snapshot is refreshed.
    
    Rate= -1: never
    Rate=  0: as often as possible
    Rate=  x: every x seconds
    """

    promptUsr("Press both buttons to pair (within 6 seconds).")

    try:
      self._wm = cwiid.Wiimote()
    except RuntimeError:
      raise WiimoteNotFoundError("No Wiimote found to pair with.")
      exit

    report("Pairing successful.")

    try:
      self._wm.enable(cwiid.FLAG_MOTIONPLUS)
    except RuntimeError:
      raise WiimoteEnableError("Found Wiimote, but could not enable it.")
      exit
      
    self.sampleRate = theSampleRate
    self._startTime = getTimeStamp();

    self._wiiCallbackStack = _WiiCallbackStack(self._wm)

    # Enable reports from the WII:
    self._wm.rpt_mode = cwiid.RPT_ACC | cwiid.RPT_MOTIONPLUS | cwiid.RPT_BTN | cwiid.RPT_IR
    
    # Initialize accelerometer calibration: get the calibration information
    # from the Wiimote. The result consists of a list of lists. The
    # first element is x/y/z of zero, the second element is x/y/z of
    # the reading at one: 
    accCalibration = self.getAccCal()
    # Tell the WIIState factory that all WIIMote state instance creations
    # should correct accelerometer readings automatically: 
    wiistate.WIIState.setAccelerometerCalibration(wiistate.WIIReading(accCalibration[0]), 
                                                  wiistate.WIIReading(accCalibration[1]))

    time.sleep(0.2)
    self._wiiCallbackStack.push(self._steadyStateCallback)

    report("Wimotion activated.")


  #----------------------------------------
  # steadyStateCallback
  #------------------

  def _steadyStateCallback(self, state, theTime):
    #print state
    now = getTimeStamp()
    if now - self._startTime >= self.sampleRate:
        self.wiiMoteState = wiistate.WIIState(state, theTime, self.getRumble(), self._wm.state['buttons']);
        self._startTime = now

  #----------------------------------------
  # zeroingCallback
  #------------------

  def _calibrationCallback(self, state, theTime):
    """Wii's callback destination while zeroing the device."""

    if self._readingsCnt >= self._NUM_ZEROING_READINGS:
      return

    thisState = wiistate.WIIState(state, theTime, self.getRumble(), self._wm.state['buttons'])

    # Pull out the accelerometer x,y,z, and build an WIIReading from them:
    self._accList.append(wiistate.WIIReading(thisState.accRaw))
    # Pull out the gyro x,y,z, and build a GyroReading from them:
    self._gyroList.append(wiistate.GyroReading(thisState.angleRate))
    self._readingsCnt += 1
    return

  #----------------------------------------
  # zero
  #------------------

  def calibrate(self):
    """Find the at-rest values of the accelerometer and the gyro.

    Collect _NUM_ZEROING_READINGS readings of acc and gyro. Average them.
    If any value is greater than the device's standard deviation,
    we conclude that user was moving the device and
    complain. Else the at-rest X,Y,Z are saved in _normalAcc and
    _normalGyro.

    """

    self._accList = []
    self._gyroList = []
    self._readingsCnt = 0
    self._wiiCallbackStack.push(self._calibrationCallback)
    
    while self._readingsCnt < self._NUM_ZEROING_READINGS:
      time.sleep(.2)
     
    self._wiiCallbackStack.pause()

    # Collected self._NUM_ZEROING_READINGS WIIReading instances for 
    # accelerometer & gyro.. Average them by building a list of 
    # all x-coords, one for all y-coords, and one for all z-coords:


    # Turn list of acc WIIState objects into list of numpy triplets:
    accArrays = map(lambda wiiReading: wiiReading.tuple(), self._accList)
    
    # Turn list of numpy triplets into three columns containing
    # all x, all y, and all z values, respectively:
    #  [array(10,20,30), array(100,200,300)] ==> [[10   20  30],
    #                                             [100 200 300]]
    # and take the means of each column. We will end up
    # with: [55.0 110.0 165.0]
    
    self.meanAcc = np.vstack(accArrays).mean(axis=0)
    self.stdevAcc = np.vstack(accArrays).std(axis=0)
    
    # Same for Gyro readings, but we need to throw out the 
    # first sample, which seems off every time:
    
    gyroArrays = map(lambda wiiReading: wiiReading.tuple(), self._gyroList[1:])
    self.meanGyro = np.vstack(gyroArrays).mean(axis=0)
    self.stdevGyro = np.vstack(gyroArrays).std(axis=0)
    
    # Initialize WIIState's gyro zero reading, so that future
    # readings can be corrected when a WIIState is created:
    wiistate.WIIState.setGyroCalibration(self.meanGyro)

    # Restore the callback that was in force before zeroing:
    self._wiiCallbackStack.pop()

    # TODO: If stdev too large, throw error
    

    return

 
  #----------------------------------------
  # setRumble
  #------------------

  def setRumble(self, switchPos):
    self._wm.rumble = switchPos


  #----------------------------------------
  # getRumble
  #------------------

  def getRumble(self):
    return self._wm.state['rumble']


  #----------------------------------------
  # setLEDs
  #------------------

  def setLEDs(self, statusList):
    """Set the four Wii LEDs according to statusList

    statusList must be a 4-tuple. Each entry
    is either True, False, or None. True will
    turn the respective LED on; False turns it
    off, and None leaves the state unchanged.

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
  # getAccCal
  #------------------

  def getAccCal(self):
    """Obtain calibration data from accelerometer.

    Retrieve factory-installed calibration data for
    the Wiimote's accelerometer. Return data format:
    dictionary with keys 'zero' and 'one'. Values
    are WIIState readings 

    """

    # Parameter is the Wiimote extension from which
    # the calibration is to be retrieved. The Nanchuk
    # adds one accelerometer. We don't have that.

    return self._wm.get_acc_cal(cwiid.EXT_NONE);
    
  #----------------------------------------
  # printState
  #------------------

  def printState(self):
      log(self.wiiMoteState)
      #**********
      #report ("Gyro raw: " + `self.wiiMoteState.rawAngleRate` + " Gyro zero: " + `self.wiiMoteState._gyroZeroReading` + " Gyro stdev: " + `self.stdevGyro`)
      #**********
    
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

