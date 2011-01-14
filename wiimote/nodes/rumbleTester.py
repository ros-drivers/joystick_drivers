#!/usr/bin/env python

# Test rumbler. Expect:
#
#  INTER_PATTERN_SLEEP_DURATION seconds of no rumble
#  Rumble: .-..
#  INTER_PATTERN_SLEEP_DURATION seconds of no rumble
#  Rumble: .-.. INTER_PATTERN_SLEEP_DURATION seconds  Rumble: .-..



import roslib; roslib.load_manifest('wiimote')
import rospy
from wiimote.msg import TimedSwitch
from wiimote.msg import RumbleControl

from wiimote.wiimoteConstants import SWITCH_ON
from wiimote.wiimoteConstants import SWITCH_OFF
from wiimote.wiimoteConstants import SWITCH_PULSE_PATTERN

morseDi    	= 0.2
morsePause 	= 0.1
morseDa    	= 0.6
morseLongPause  = 1.0
    
INTER_PATTERN_SLEEP_DURATION = 3.0

def printMsg(msg):
  pattern = msg.rumble.pulse_pattern
  print "switch_mode: %2d; num_cycles: %3d, pulse_pattern:\n[" % (msg.rumble.switch_mode, msg.rumble.num_cycles)
  for duration in pattern:
    print "%.2f, " % (duration),
  print "]"
    
  

def talker():

    # Send one message, or keep repeating?
    oneShot = True
    
    pub = rospy.Publisher('/wiimote/rumble', RumbleControl)
    rospy.init_node('rumbleTester', anonymous=True)


    sendBlank(pub)
    rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

    # ------------------

    sendOneMorse(pub)
    rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

    # ------------------
    
    sendTwoMorse(pub)
    rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

    #rospy.sleep(FINAL_SLEEP_DURATION)


def sendBlank(pub):

    print("Ask wiimote node for pattern, but set num_cycles=0. ==> No rumble")
    msg = RumbleControl(TimedSwitch(switch_mode=SWITCH_PULSE_PATTERN,
				    num_cycles = 0,
				    pulse_pattern=[morseDi,
						   morsePause,
						   morseDa,
						   morsePause,
						   morseDi,
						   morsePause,
						   morseDi,
						   morseLongPause]))

    printMsg(msg)
    pub.publish(msg)
  

def sendOneMorse(pub):

    print("Ask wiimote node for pattern, set num_cycles=1. ==> One Morse '. - . .' rumble")
    msg = RumbleControl(TimedSwitch(switch_mode=SWITCH_PULSE_PATTERN,
				    num_cycles = 1,
				    pulse_pattern=[morseDi,
						   morsePause,
						   morseDa,
						   morsePause,
						   morseDi,
						   morsePause,
						   morseDi,
						   morseLongPause]))

    printMsg(msg)
    pub.publish(msg)
  

def sendTwoMorse(pub):

    print("Ask wiimote node for rumble pattern, set num_cycles=2. ==> Two Morse '. - . .' rumble cycles")
    msg = RumbleControl(TimedSwitch(switch_mode=SWITCH_PULSE_PATTERN,
				    num_cycles = 2,
				    pulse_pattern=[morseDi,
						   morsePause,
						   morseDa,
						   morsePause,
						   morseDi,
						   morsePause,
						   morseDi,
						   morseLongPause]))

    printMsg(msg)
    pub.publish(msg)
  

if __name__ == '__main__':
    try:
        print("\n   ****************************************************************************************\n")
        print("**** You should feel three repeated vibrations, spelling out Morse code pattern for 'L': .-.. *****\n")
        print("Console messages below describe ROS messages being sent.")
        print("\n   ****************************************************************************************\n\n")
        talker()
    except rospy.ROSInterruptException: pass
