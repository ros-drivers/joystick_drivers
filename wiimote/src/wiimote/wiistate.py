################################################################################
#
# File:         wiistate.py
# RCS:          $Header: $
# Description:  Object representation of Wiimote's state
# Author:       Andreas Paepcke
# Created:      Thu Aug 13 11:01:34 2009
# Modified:     Thu Aug 20 08:46:24 2009 (Andreas Paepcke) paepcke@anw.willowgarage.com
# Language:     Python
# Package:      N/A
# Status:       Experimental (Do Not Distribute)
#
# 
#
################################################################################

from wiimoteConstants import *
from wiiutils import *
import numpy as np
#import scipy as sp

#import matplotlib as mpl
#import matplotlib.pyplot as ppl



#----------------------------------------
# Class WIIState
#---------------

class WIIState(object):
  """Holds the state of a WIIRemote-plus.

  The state is passed in and is as communicated
  by one message from the WII+ device. We unpack 
  the information and place it into individual 
  dictionaries for callers to grab.

  """

  _accCalibrationZero = None   # Will be a WIIReading
  _accCalibrationOne  = None   # Will be a WIIReading
  _gyroZeroReading    = None   # Will be the gyro's zero-movement reading 
#  time      = 0
#  ascTime   = ""
#  angleRate = GyroReading()
#  acc       = WIIReading()
#  
#  buttons   = {BTN_1: False, BTN_2: False, BTN_PLUS: False,
#               BTN_MINUS: False, BTN_A: False, BTN_B: False,
#               BTN_UP: False, BTN_DOWN: False, BTN_LEFT: False,
#               BTN_RIGHT: False, BTN_HOME: False}
#  IRSources = {IR1:None, IR2:None, IR3:None, IR4:None}
#  battery   = None
#  rumble    = False
#

  #----------------------------------------
  # __init__
  #----------

  def __init__(self, state, theTime, theRumble, buttonStatus):
    """Unpack the given state, normalizing if normalizers are passed in."""

    self.time = theTime
    ## self.ascTime = time.asctime(time.gmtime(theTime))
    self.ascTime = `theTime`
    self.rumble = theRumble
    self.IRSources = {IR1:None, IR2:None, IR3:None, IR4:None}
    self.battery = None
    self.angleRate = None
    self.buttons   = {BTN_1: False, BTN_2: False, BTN_PLUS: False,
                      BTN_MINUS: False, BTN_A: False, BTN_B: False,
                      BTN_UP: False, BTN_DOWN: False, BTN_LEFT: False,
                      BTN_RIGHT: False, BTN_HOME: False}
    self.accRaw = None
    self.angleRate = None
    
    # Handle buttons on the WII
    # A zero means no button is down.

    if buttonStatus == 0:
      for key in self.buttons.keys():
        self.buttons[key] = False
        continue

    self.buttons[BTN_1]     = (buttonStatus & BTN_1) > 0
    self.buttons[BTN_2]     = (buttonStatus & BTN_2) > 0
    self.buttons[BTN_PLUS]  = (buttonStatus & BTN_PLUS) > 0
    self.buttons[BTN_MINUS] = (buttonStatus & BTN_MINUS) > 0
    self.buttons[BTN_A]     = (buttonStatus & BTN_A) > 0
    self.buttons[BTN_B]     = (buttonStatus & BTN_B) > 0
    self.buttons[BTN_UP]    = (buttonStatus & BTN_UP) > 0
    self.buttons[BTN_DOWN]  = (buttonStatus & BTN_DOWN) > 0
    self.buttons[BTN_LEFT]  = (buttonStatus & BTN_LEFT) > 0
    self.buttons[BTN_RIGHT] = (buttonStatus & BTN_RIGHT) > 0
    self.buttons[BTN_HOME]  = (buttonStatus & BTN_HOME) > 0

    for msgComp in state:
      # msgComp has: (1,2) for Status:Button one pushed, or
      #              (3, [None,None,None,None]) for LEDs
      #              (7, {'angle_rage': (123,456,789)})
      msgType = msgComp[0]

      if msgType == WII_MSG_TYPE_ACC:

        # Second list member is accelerator triplet of numbers:
        accStatus = msgComp[1]
        self.accRaw = WIIReading(accStatus, self.time)
        
        # If this class knows about accelerometer calibration
        # data, correct the raw reading:
        if self._accCalibrationZero is not None and self._accCalibrationOne is not None:
            self.acc = WIIReading((self.accRaw - self._accCalibrationZero) / (self._accCalibrationOne - self._accCalibrationZero), self.time)
        else:
            self.acc = self.accRaw

        continue

      elif msgType == WII_MSG_TYPE_IR:

        # Second list member is a list of 4 elements,
        # one for each of the Wii LEDs:

        IRStatus = msgComp[1]
        self.IRSources[IR1] = IRStatus[0]
        self.IRSources[IR2] = IRStatus[1]
        self.IRSources[IR3] = IRStatus[2]
        self.IRSources[IR4] = IRStatus[3]

        continue

      elif msgType == WII_MSG_TYPE_MOTIONPLUS:

        # Second list member is a dictionary with the single
        # key 'angle_rate', which yields as its value a gyro 
        # readings triplet of numbers:

        gyroDict = msgComp[1]
        
        if gyroDict is not None:
            # If this class knows about a zero-movement reading for the
            # gyro, subtract that reading from the raw measurement:
            
            self.rawAngleRate = gyroDict['angle_rate']
            if self._gyroZeroReading is not None:
                self.angleRate = GyroReading(self.rawAngleRate - self._gyroZeroReading, self.time)
            else:
                self.angleRate = GyroReading(self.rawAngleRate, self.time)

        continue
       

  #----------------------------------------
  # setAccelerometerCalibration
  #----------
  
  @classmethod
  def setAccelerometerCalibration(cls, zeroReading, oneReading):
      cls._accCalibrationZero = zeroReading
      cls._accCalibrationOne = oneReading

  #----------------------------------------
  # setGyroCalibration
  #----------
  
  @classmethod
  def setGyroCalibration(cls, zeroReading):
      cls._gyroZeroReading = zeroReading

#----------------------------------------
  # __str___
  #----------

  def __str__(self):

    # Timestamp:
    res = 'Time: ' + self.ascTime + '\n'

    # Buttons that are pressed:
    butRes = ''

    if self.buttons is not None:
        if self.buttons[BTN_1]:
          butRes += ', 1'
        if self.buttons[BTN_2]:
          butRes += ', 2'
        if self.buttons[BTN_PLUS]:
          butRes += ', Plus'
        if self.buttons[BTN_MINUS]:
          butRes += ', Minus'
        if self.buttons[BTN_A]:
          butRes += ', A'
        if self.buttons[BTN_B]:
          butRes += ', B'
        if self.buttons[BTN_UP]:
          butRes += ', 4Way-up'
        if self.buttons[BTN_DOWN]:
          butRes += ', 4Way-down'
        if self.buttons[BTN_LEFT]:
          butRes += ', 4Way-left'
        if self.buttons[BTN_RIGHT]:
          butRes += ', 4Way-right'
        if self.buttons[BTN_HOME]:
          butRes += ', Home'


    # If none of the buttons is down, indicate that:
    if not butRes:
      res += 'Buttons: none.\n'
    else:
      res += 'Buttons: ' + butRes.lstrip(', ') + '\n'
      

    # Accelerator:
    if self.acc is not None:
        res += 'Accelerator: (' + \
               `self.acc[X]` + ',' + \
               `self.acc[Y]` + ',' + \
               `self.acc[Z]` + ')\n'
        
    # Gyro (angular rate):

    if self.angleRate is not None:
        res += 'Gyro (angular rate): (' + \
               `self.angleRate[X]` + ',' + \
               `self.angleRate[Y]` + ',' + \
               `self.angleRate[Z]` + ')\n'
    
    # Rumble status:

    if self.rumble:
      res += 'Rumble: On.\n'
    else:
      res += 'Rumble: Off.\n'

    # IR Sources:

    irRes = '' 

    if self.IRSources is not None:
        if self.IRSources[IR1] is not None:
          irRes += 'IR source 1'
    
        if self.IRSources[IR2] is not None:
          irRes += 'IR source 2'
    
        if self.IRSources[IR3] is not None:
          irRes += 'IR source 3'
    
        if self.IRSources[IR4] is not None:
          irRes += 'IR source 4'
    
        if not irRes:
          res += irRes.lstrip(', ') + '\n'
        else:
          res += 'No IR sources detected.\n'


    return res


#----------------------------------------
# Class WIIReading
#-----------------

class WIIReading(object):
  """Instances hold one 3-D reading.

  Methods:
    [X], [Y], [Z] to obtain respective axis paramters.
    tuple() to obtain x/y/z as a NumPy array.
    +,-,/ to add or subtract readings from each other
        as one vector operation (pairwise for each dimension).
  """

  #_measurement = np.array([0,0,0])
  #time = None


  def __init__(self, xyz, theTime=None):
    """Create a (possibly) time stamped WII Reading.
    
    Parameter xyz is an array of x,y,z coordinates of the
    reading. WIIReading instances can be added, subtracted, and
    divided into each other. The operations are pairwise over
    x, y, and z. A numpy array of x,y,z is available by
    calling tuple(). The time stamp is available via time().
    
    """ 
    self.time = theTime
    self._measurement = np.array([xyz[X], xyz[Y], xyz[Z]])

  def __getitem__(self, key):
    if key not in (X,Y,Z):
        raise AttributeError("Attempt to index into a 3-D measurement array with index " + `key` + ".")
    return self._measurement[key]

  def tuple(self):
    return self._measurement

  def __add__(self, other):
    """Adding two readings returns a numpy tuple with readings added pairwise."""

    return self._measurement + other._measurement

  def __sub__(self, other):
    """Subtracting two readings returns a numpy tuple with components subtracted pairwise."""

    return self._measurement - other._measurement

  def __div__(self, other):
    """Dividing two readings returns a numpy tuple with components divided pairwise."""

    return self._measurement / other._measurement

#----------------------------------------
# Class GyroReading
#------------------

# TODO: modify GyroReading to be in terms of angular units.
class GyroReading():
  """Instances hold one gyroscope reading.

      Methods:
        [PHI], [THETA], [PSI] to obtain respective axis paramters.
        tuple() to obtain phi/theta/psi as a NumPy array.
        +,-,/ to add or subtract readings from each other
        as one vector operation (pairwise for each dimension).
  """
  # _measurement = np.array([0,0,0])
  #time = None


  def __init__(self, phiThetaPsi, theTime=None):
    """Create a (possibly) time stamped WII Reading.
    
    Parameter phiThetaPsi is an array of phi,theta,psi coordinates of the
    gyro reading. GyroReading instances can be added, subtracted, and
    divided into each other. The operations are pairwise over
    phi, theta, and psi. A numpy array of phi,theta,psi is available by
    calling tuple(). The time stamp is available via time().
    self.time = theTime

    """
    
    self._measurement = np.array([phiThetaPsi[PHI], phiThetaPsi[THETA], phiThetaPsi[PSI]])
    

  def __getitem__(self, key):
    if key not in (PHI,THETA,PSI):
        raise AttributeError("Attempt to index into a 3-D measurement array with index " + `key` + ".")
    return self._measurement[key]

  def tuple(self):
    return self._measurement

  def __add__(self, other):
    """Adding two gyro readings returns a new reading with components added pairwise."""
    return self._measurement + other._measurement

  def __sub__(self, other):
    """Subtracting two gyro readings returns a new reading 
    with components subtracted pairwise.

    """
    return self._measurement - other._measurement

  def __div__(self, other):
    """Dividing two readings returns a numpy tuple with components divided pairwise."""

    return self._measurement / other._measurement

#;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
#
#    Utility Functions
#
#;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

