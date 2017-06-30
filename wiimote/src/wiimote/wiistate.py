################################################################################
#
# File:         wiistate.py
# RCS:          $Header: $
# Description:  Object representation of Wiimote's state
# Author:       Andreas Paepcke
# Created:      Thu Aug 13 11:01:34 2009 (Andreas Paepcke) paepcke@anw.willowgarage.com
# Modified:     Thu Jan 13 13:50:15 2011 (Andreas Paepcke) paepcke@bhb.willowgarage.com
# Language:     Python
# Package:      N/A
# Status:       Experimental (Do Not Distribute)
#
# 
#
################################################################################
#
# Revisions:
#
# Thu Mar 18 10:56:09 2010 (David Lu) davidlu@wustl.edu
#  Added nunchuk state variables
# Fri Oct 29 08:58:21 2010 (Miguel Angel Julian Aguilar, QBO Project) miguel.angel@thecorpora.com
#  Added classic controller state variables
# Mon Nov 08 11:43:23 2010 (David Lu) davidlu@wustl.edu
#  Added calibration for nunchuk
################################################################################

from wiimoteConstants import *
from wiiutils import *
import numpy as np

#----------------------------------------
# Class WIIState
#---------------

class WIIState(object):
  """Holds the state of a WIIRemote-plus.

      The state is passed in and is as communicated
      by one message from the WII+ device. We unpack 
      the information and place it into individual 
      dictionaries for callers to grab.
      
      Public instance variables:
        o time             Time in fractional seconds since beginning of Epoch of when 
                             state was measured (Float).
        o ascTime          Time when state was measured (Human-readable)
        o rumble           True/False if wiimote vibration is on/off
        o angleRate        A GyroReading instance containing gyro (a.k.a. angular rate) measurement
        o acc              A WIIReading instance containing accelerometer measurement corrected by
                             the calibration information that is stored in the Wiimote
        o accRaw           A WIIReading instance containing accelerometer measurement uncorrected
        o buttons          A dictionary for which buttons are being held down. That could be
                             multiple buttons. Keys are:
                                   BTN_1, BTN_2, BTN_PLUS, BTN_MINUS, BTN_A, BTN_B,
                                   BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT, BTN_HOME
                             Values are 1/0
        o IRSources        Dictionary with on/off values for which IR lights are
                           being sensed. Keys are:
                                   IR1, IR2, IR3, IR4
                           Values are 1/0
        o motionPlusPresent True if a gyro Motion+ is plugged into the Wiimote. Else False

        o nunchukPresent   True if nunchuk is plugged in. Else False
        o nunchukAccRaw    A WIIReading instance with acceleromoter measurement from the nunchuk (raw values)
        o nunchukAcc       The same, but zeroed using factory calibration
        o nunchukStickRaw  A tuple with the two axes of the joystick on the nunchuk, raw readings
        o nunchukStick     A tuple with the two axes of the joystick on the nunchuk, zeroed to be [-1, 1]
        o nunchukButtons   A dictionary for which nunchuk buttons are down. Keys are BTN_C and BTN_Z
  
      Public methods:
        o setAccelerometerCalibration   Bias setting for accelerometer. This triplet is used to
                                          turn raw accelerometer values into calibrated values.
        o setGyroCalibration            Bias setting for gyro. This triplet is used to
                                          turn raw gyro values into calibrated values.
  """
  
  _accCalibrationZero = None
  _gyroZeroReading = None
  _nunchukZeroReading = None
  _nunchukJoystickZero = None

  #----------------------------------------
  # __init__
  #----------

  def __init__(self, state, theTime, theRumble, buttonStatus):
    """Unpack the given state, normalizing if normalizers are passed in."""

    self.time = theTime
    self.ascTime = `theTime`
    self.rumble = theRumble
    self.IRSources = [None, None, None, None]
    self.battery = None
    self.acc = None
    self.accRaw = None
    self.angleRate = None
    self.angleRate = None
    self.angleRageRaw = None
    self.motionPlusPresent = False
    self.buttons   = {BTN_1: False, BTN_2: False, BTN_PLUS: False,
                      BTN_MINUS: False, BTN_A: False, BTN_B: False,
                      BTN_UP: False, BTN_DOWN: False, BTN_LEFT: False,
                      BTN_RIGHT: False, BTN_HOME: False}
    self.nunchukPresent = False
    self.nunchukAccRaw = None
    self.nunchukAcc = None
    self.nunchukStick = None
    self.nunchukStickRaw = None
    self.nunchukButtons = {BTN_C: False, BTN_Z: False}

    self.classicPresent = False
    self.classicStickLeft = None
    self.classicStickRight = None
    self.classicButtons = {CLASSIC_BTN_A: False, CLASSIC_BTN_B: False,
                           CLASSIC_BTN_L: False, CLASSIC_BTN_R: False,
                           CLASSIC_BTN_X: False, CLASSIC_BTN_Y: False,
                           CLASSIC_BTN_ZL: False, CLASSIC_BTN_ZR: False,
                           CLASSIC_BTN_PLUS: False, CLASSIC_BTN_MINUS: False,
                           CLASSIC_BTN_UP: False, CLASSIC_BTN_DOWN: False,
                           CLASSIC_BTN_LEFT: False, CLASSIC_BTN_RIGHT: False,
                           CLASSIC_BTN_HOME: False}

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
        if self._accCalibrationZero is not None and self._accCalibrationOne is not None and np.linalg.norm(self._accCalibrationOne - self._accCalibrationZero) > 1E-5:
            self.acc = WIIReading((self.accRaw - self._accCalibrationZero) / (self._accCalibrationOne - self._accCalibrationZero), self.time)
        else:
            self.acc = WIIReading(self.accRaw, self.time)

        continue

      elif msgType == WII_MSG_TYPE_IR:
        # Second list member is a list of 4 dictionaries,
        # one for each of the Wii IR sources:

        IRStatus = msgComp[1]
        # IRStatus is an array of four Dicts. Ex: [{'pos': (317, 445)}, None, None, None]
        self.IRSources[0] = IRStatus[0]
        self.IRSources[1] = IRStatus[1]
        self.IRSources[2] = IRStatus[2]
        self.IRSources[3] = IRStatus[3]
        
        continue

      elif msgType == WII_MSG_TYPE_MOTIONPLUS:
        # Second list member is a dictionary with the single
        # key 'angle_rate', which yields as its value a gyro 
        # readings triplet of numbers:

        gyroDict = msgComp[1]
        
        if gyroDict is not None:
            # If this class knows about a zero-movement reading for the
            # gyro, subtract that reading from the raw measurement:
            
            self.angleRateRaw = GyroReading(gyroDict['angle_rate'], self.time)
            if self._gyroZeroReading is not None:
                self.angleRate = GyroReading(self.angleRateRaw - self._gyroZeroReading, self.time)
            else:
                self.angleRate = self.angleRateRaw
                
            self.motionPlusPresent = True

        continue
      elif msgType == WII_MSG_TYPE_NUNCHUK:
        nunChuk = msgComp[1];
        if nunChuk is not None:
            self.nunchukPresent = True
            self.nunchukAccRaw = WIIReading(nunChuk['acc'], self.time)
        
            # If this class knows about accelerometer calibration
            # data, correct the raw reading:
            if self._nunchukZeroReading is not None:
                self.nunchukAcc = WIIReading((self.nunchukAccRaw - self._nunchukZeroReading) / (self._nunchukOneReading - self._nunchukZeroReading), self.time)
            else:
                self.nunchukAcc = WIIReading(self.nunchukAccRaw, self.time)


            self.nunchukStickRaw = nunChuk['stick']
            
            # scale the joystick to roughly [-1, 1] 
            if (self._nunchukJoystickZero is None):
                calibration = [127, 127]
            else:
                calibration = self._nunchukJoystickZero

            [joyx, joyy] = self.nunchukStickRaw
            joyx = -(joyx-calibration[0])/100.
            joyy = (joyy-calibration[1])/100.
            # create a deadzone in the middle
            if abs(joyx) < .05:
                joyx = 0
            if abs(joyy) < .05:
                joyy = 0
            self.nunchukStick = [joyx,joyy]

            nunButtons = nunChuk['buttons']
            self.nunchukButtons[BTN_C]  = (nunButtons & BTN_C) > 0
            self.nunchukButtons[BTN_Z]  = (nunButtons & BTN_Z) > 0
        continue
      elif msgType == WII_MSG_TYPE_CLASSIC:
        clasSic = msgComp[1];
        if clasSic is not None:
            self.classicPresent = True
            self.classicStickLeft = clasSic['l_stick']
            self.classicStickRight = clasSic['r_stick']
            clasButtons = clasSic['buttons']
            self.classicButtons[CLASSIC_BTN_A]  = (clasButtons & CLASSIC_BTN_A) > 0
            self.classicButtons[CLASSIC_BTN_B]  = (clasButtons & CLASSIC_BTN_B) > 0
            self.classicButtons[CLASSIC_BTN_DOWN]  = (clasButtons & CLASSIC_BTN_DOWN) > 0
            self.classicButtons[CLASSIC_BTN_HOME]  = (clasButtons & CLASSIC_BTN_HOME) > 0
            self.classicButtons[CLASSIC_BTN_L]  = (clasButtons & CLASSIC_BTN_L) > 0
            self.classicButtons[CLASSIC_BTN_LEFT]  = (clasButtons & CLASSIC_BTN_LEFT) > 0
            self.classicButtons[CLASSIC_BTN_MINUS]  = (clasButtons & CLASSIC_BTN_MINUS) > 0
            self.classicButtons[CLASSIC_BTN_PLUS]  = (clasButtons & CLASSIC_BTN_PLUS) > 0
            self.classicButtons[CLASSIC_BTN_R]  = (clasButtons & CLASSIC_BTN_R) > 0
            self.classicButtons[CLASSIC_BTN_RIGHT]  = (clasButtons & CLASSIC_BTN_RIGHT) > 0
            self.classicButtons[CLASSIC_BTN_UP]  = (clasButtons & CLASSIC_BTN_UP) > 0
            self.classicButtons[CLASSIC_BTN_X]  = (clasButtons & CLASSIC_BTN_X) > 0
            self.classicButtons[CLASSIC_BTN_Y]  = (clasButtons & CLASSIC_BTN_Y) > 0
            self.classicButtons[CLASSIC_BTN_ZL]  = (clasButtons & CLASSIC_BTN_ZL) > 0
            self.classicButtons[CLASSIC_BTN_ZR]  = (clasButtons & CLASSIC_BTN_ZR) > 0
        continue
       

  #----------------------------------------
  # setAccelerometerCalibration
  #----------
  
  @classmethod
  def setAccelerometerCalibration(cls, zeroReading, oneReading):
      """Set the current accelerometer zeroing calibration."""
      cls._accCalibrationZero = WIIReading(zeroReading)
      cls._accCalibrationOne = WIIReading(oneReading)

  #----------------------------------------
  # getAccelerometerCalibration
  #----------
  
  @classmethod
  def getAccelerometerCalibration(cls):
      """Return current accelerometer zeroing offset as two lists of x/y/z: the 
      zero-reading, and the one-reading."""
      return (cls._accCalibrationZero.tuple(), cls._accCalibrationOne.tuple())

  #----------------------------------------
  # setGyroCalibration
  #----------
  
  @classmethod
  def setGyroCalibration(cls, zeroReading):
      """Set the x/y/z zeroing offsets for the gyro. Argument is a list"""
      
      cls._gyroZeroReading = GyroReading(zeroReading)

  #----------------------------------------
  # getGyroCalibration
  #----------
  
  @classmethod
  def getGyroCalibration(cls):
      """Return current gyro zeroing offset as a list of x/y/z. """
      return cls._gyroZeroReading.tuple()

  #----------------------------------------
  # setNunchukAccelerometerCalibration
  #----------
  
  @classmethod
  def setNunchukAccelerometerCalibration(cls, zeroReading, oneReading):
      """Set the current nunchuk accelerometer zeroing calibration."""
      cls._nunchukZeroReading = WIIReading(zeroReading)
      cls._nunchukOneReading = WIIReading(oneReading)

  #----------------------------------------
  # setNunchukJoystickCalibration
  #----------
  
  @classmethod
  def setNunchukJoystickCalibration(cls, readings):
      """Set the origin for the nunchuk joystick"""
      cls._nunchukJoystickZero = readings    

  #----------------------------------------
  # getNunchukAccelerometerCalibration
  #----------
  
  @classmethod
  def getNunchukAccelerometerCalibration(cls):
      """Return current nunchuk accelerometer zeroing offset as two lists of x/y/z: the 
      zero-reading, and the one-reading."""
      return (cls._nunchukZeroReading.tuple(), cls._nunchukOneReading.tuple())


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
  # __repr___
  #----------

  def __repr__(self):
      return self.__str__()


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

  # Private instance vars:
  #     o _measurement = np.array(3, dtype=numpy.float64)
  #     o time


  def __init__(self, xyz, theTime=None):
    """Create a (possibly) time stamped WII Reading.
    
    Parameter xyz is an array of x,y,z coordinates of the
    reading. WIIReading instances can be added, subtracted, and
    divided into each other. The operations are pairwise over
    x, y, and z. A numpy array of x,y,z is available by
    calling tuple(). The time stamp is available via time().
    
    """ 
    self.time = theTime
    self._measurement = np.array([xyz[X], xyz[Y], xyz[Z]],dtype=np.float64)

  def __getitem__(self, key):
    if key not in (X,Y,Z):
        raise AttributeError("Attempt to index into a 3-D measurement array with index " + `key` + ".")
    return self._measurement[key]

    def __str__(self):
      return '[x=' + repr(self._measurement[X]) + \
             ', y=' + repr(self._measurement[Y]) + \
             ' z=' + repr(self._measurement[Z]) + \
             ']'

  def __repr__(self):
      return '[' + str(self._measurement[X]) + ', ' + str(self._measurement[Y]) + ', ' + str(self._measurement[Z]) + ']'
  
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

  def scale(self, scaleFactor):
    """Return a numpy tuple that with X, Y, Z scaled by the given factor."""
    
    return self._measurement * scaleFactor

#----------------------------------------
# Class GyroReading
#------------------

class GyroReading():
  """Instances hold one gyroscope reading.

      Methods:
        [PHI], [THETA], [PSI] to obtain respective axis paramters.
        tuple() to obtain phi/theta/psi as a NumPy array.
        +,-,/ to add or subtract readings from each other
        as one vector operation (pairwise for each dimension).
  """
  # Local instance vars:
  #   o _measurement = np.array(3, dtype=numpy.float64)
  #   o time 


  def __init__(self, phiThetaPsi, theTime=None):
    """Create a (possibly) time stamped WII Reading.
    
    Parameter phiThetaPsi is an array of phi,theta,psi coordinates of the
    gyro reading. GyroReading instances can be added, subtracted, and
    divided into each other. The operations are pairwise over
    phi, theta, and psi. A numpy array of phi,theta,psi is available by
    calling tuple(). The time stamp is available via time().
    """

    self.time = theTime
    self._measurement = np.array([phiThetaPsi[PHI], phiThetaPsi[THETA], phiThetaPsi[PSI]],dtype=np.float64)
    

  def __getitem__(self, key):
    if key not in (PHI,THETA,PSI):
        raise AttributeError("Attempt to index into a 3-D measurement array with index " + `key` + ".")
    return self._measurement[key]

  def __str__(self):
    return '[PHI (roll)=' + repr(self._measurement[PHI]) + \
           ', THETA (pitch)=' + repr(self._measurement[THETA]) + \
           ', PSI (yaw)=' + repr(self._measurement[PSI]) + \
           ']'

  def __repr__(self):
      '[' + str(self._measurement[PHI]) + ', ' + str(self._measurement[THETA]) + ', ' + str(self._measurement[PSI]) + ']'
      
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

  def scale(self, scaleFactor):
    """Return a numpy tuple that with X, Y, Z scaled by the given factor."""
    
    return self._measurement * scaleFactor

#;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
#
#    Utility Functions
#
#;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
