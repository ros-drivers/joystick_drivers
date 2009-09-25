#!/usr/bin/python
#;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
#
#    WIIMote Run Tests
#
#;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

# TODO: Don't duplicate cwiid constants in armConrolConstants.py
# TODO: Offer exception if stdev is too high in IMU readings
# TODO: Zeroing
# TODO: Catch wiimote being turned off.


import WIIMote
from wiimoteExceptions import *
from wiiutils import *
from wiistate import *
import numpy as np 

mySampleRate = 1
try:
  ac = WIIMote.WIIMote(theSampleRate = mySampleRate)
  ac.zeroDevice()

  #********************
#  numSamples = 50
#
#  accMeans  = []
#  accStdevs = []
#  gyroMeans = []
#  gyroStdevs = []
#  
#  for i in range(numSamples):
#      ac.zero()
#      accMeans.append(ac.meanAcc)
#      accStdevs.append(ac.stdevAcc)
#      gyroMeans.append(ac.meanGyro)
#      gyroStdevs.append(ac.stdevGyro)
#      
#  accMean =   np.vstack(accMeans).mean(axis=0)
#  accStdev =  np.vstack(accMeans).std(axis=0)
#  gyroMean =   np.vstack(gyroMeans).mean(axis=0)
#  gyroStdev =  np.vstack(gyroMeans).std(axis=0)
#  
#  report("Acc mean: " + `accMean` + " Min mean: " + `np.vstack(accMeans).min(axis=0)` + " Max mean: " + `np.vstack(accMeans).max(axis=0)`)
#  report("Acc stdev: " + `np.vstack(accMeans).std(axis=0)`)
#  
#  report("Gyro mean: " + `gyroMean` + " Min gyro: " + `np.vstack(gyroMeans).min(axis=0)` + " Max gyro: " + `np.vstack(gyroMeans).max(axis=0)`)
#  report("Gyro stdev: " + `gyroStdev`)
#
#  
#  
#  
#  exit(0)      
#  
#  report ("Acc list: " + `map(lambda wiiReading: wiiReading.tuple(), ac._accList)`)
#  report ("Acc mean: " + `ac.meanAcc` + " Acc stdev: " + `ac.stdevAcc`)
#  report ("Gyro list: " + `map(lambda wiiReading: wiiReading.tuple(), ac._gyroList[1:])`)
#  report ("Gyro mean: " + `ac.meanGyro` + " Gyro stdev: " + `ac.stdevGyro`)
  #********************
  
  #ac.integrate()
except WiimoteError, e:
  report(e)
  exit()

ledCycle = 0
try:
  while True:
    #time.sleep(.2)
    time.sleep(mySampleRate)
    
    ac.printState()

    if ledCycle == 0:
      ac.setLEDs([True, False, False, False])  
    elif ledCycle == 1:
      ac.setLEDs([False, True, False, False])
    elif ledCycle == 2:
      ac.setLEDs([False, False, True, False])
    elif ledCycle == 3:
      ac.setLEDs([False, False, False, True])
    elif ledCycle == 4:
      ac.setLEDs([True, False, False, None])
    elif ledCycle == 5:
      ac.setLEDs([None, True, True, None])
    elif ledCycle == 6:
      ac.setLEDs([False, False, False, False])
    ledCycle += 1
    if ledCycle >= 7:
      ledCycle = 0
      #******ac.setRumble(not ac.getRumble())


finally:
  ac.shutdown()
