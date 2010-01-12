#!/usr/bin/env python

import roslib; roslib.load_manifest('wiimote')
import rospy
from wiimote.msg import LEDControl
from wiimote.msg import TimedSwitch
from wiimote.msg import State


# Test the wiimote package LED control. Expect the following LED
# patterns of INTER_PATTERN_SLEEP_DURATION seconds duration each:


INTER_PATTERN_SLEEP_DURATION = 2.0

#  1 0 0 0
#  0 1 0 0
#  0 0 1 0
#  0 0 0 1
#  0 1 1 1
#  1 0 0 1
#  All LEDs together blinking Morse code L: .-.. twice.


def talker():

    # Send one message, or keep repeating?
    oneShot = False
    
    morseDi            = 0.2
    morsePause         = 0.1
    morseDa            = 0.6
    morseLongPause  = 1.0
    
    pub = rospy.Publisher('/wiimote/leds', LEDControl)
    rospy.init_node('ledControlTester', anonymous=True)
    
    onSwitch       = TimedSwitch(switch_mode=TimedSwitch.ON)
    offSwitch      = TimedSwitch(switch_mode=TimedSwitch.OFF)
    noChangeSwitch = TimedSwitch(switch_mode=TimedSwitch.NO_CHANGE)

    msg0 = LEDControl(timed_switch_array=[offSwitch, offSwitch, offSwitch, offSwitch])
    msg1 = LEDControl(timed_switch_array=[onSwitch, offSwitch, offSwitch, offSwitch])
    msg2 = LEDControl(timed_switch_array=[offSwitch, onSwitch, offSwitch, offSwitch])
    msg3 = LEDControl(timed_switch_array=[offSwitch, offSwitch, onSwitch, offSwitch])
    msg4 = LEDControl(timed_switch_array=[offSwitch, offSwitch, offSwitch, onSwitch])
    msg5 = LEDControl(timed_switch_array=[offSwitch, onSwitch, onSwitch, onSwitch])
    msg6 = LEDControl(timed_switch_array=[onSwitch, offSwitch, offSwitch, noChangeSwitch])    


    msg7 = LEDControl(
              timed_switch_array=[TimedSwitch(switch_mode=TimedSwitch.REPEAT,
                              num_cycles = 2,
                              pulse_pattern=[morseDi,
                                     morsePause,
                                     morseDa,
                                     morsePause,
                                     morseDi,
                                     morsePause,
                                     morseDi,
                                     morseLongPause]),
                      TimedSwitch(switch_mode=TimedSwitch.REPEAT,
                              num_cycles = 2,
                              pulse_pattern=[morseDi,
                                     morsePause,
                                     morseDa,
                                     morsePause,
                                     morseDi,
                                     morsePause,
                                     morseDi,
                                     morseLongPause]),
                      TimedSwitch(switch_mode=TimedSwitch.REPEAT,
                              num_cycles = 2,
                              pulse_pattern=[morseDi,
                                     morsePause,
                                     morseDa,
                                     morsePause,
                                     morseDi,
                                     morsePause,
                                     morseDi,
                                     morseLongPause]),
                      TimedSwitch(switch_mode=TimedSwitch.REPEAT,
                              num_cycles = 2,
                              pulse_pattern=[morseDi,
                                     morsePause,
                                     morseDa,
                                     morsePause,
                                     morseDi,
                                     morsePause,
                                     morseDi,
                                     morseLongPause])])
                      

    while not rospy.is_shutdown():

      if msg0 is not None:
          rospy.logdebug("Msg0: " + str(msg0))
          pub.publish(msg0)
          rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

      if msg1 is not None:
          rospy.logdebug("Msg1: " + str(msg1))
          pub.publish(msg1)
          rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

      if msg2 is not None:
          rospy.logdebug("Msg2: " + str(msg2))
          pub.publish(msg2)
          rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

      if msg3 is not None:
          rospy.logdebug("Msg3: " + str(msg3))
          pub.publish(msg3)
          rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

      if msg4 is not None:
          rospy.logdebug("Msg4: " + str(msg4))
          pub.publish(msg4)
          rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

      if msg5 is not None:
          rospy.logdebug("Msg5: " + str(msg5))
          pub.publish(msg5)
          rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

      if msg6 is not None:
          rospy.logdebug("Msg6: " + str(msg6))
          pub.publish(msg6)
          rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

      if msg7 is not None:
          rospy.logdebug("Msg7: " + str(msg7))
          pub.publish(msg7)
          rospy.sleep(INTER_PATTERN_SLEEP_DURATION)


if __name__ == '__main__':
    
  print("\n   ****************************************************************\n")
  print("****     You should see seven LED on/off configurations,  ****")
  print("        plus a twice-repeated, all-ligths Morse code 'L' (.-..)") 
  print("\n   **************************************************************")
  print("[off, off, off, off]")
  print("[on,  off, off, off]")
  print("[off, on,  off, off]")
  print("[off, off, on,  off]")
  print("[off, off, off, on ]")
  print("[off, on,  on,  on ]")
  print("[on,  off, off, on ]")
  print("Twice all LEDs flashing a Morse 'L': .-..\n")
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
