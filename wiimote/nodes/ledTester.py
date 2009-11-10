#!/usr/bin/env python

import roslib; roslib.load_manifest('wiimote')
import rospy
from wiimote.msg import LEDControl
from wiimote.msg import TimedSwitch


# Test the wiimote package LED control. Expect the following LED
# patterns of INTER_PATTERN_SLEEP_DURATION seconds duration each:


INTER_PATTERN_SLEEP_DURATION = 2.0

#  1 0 0 0
#  0 1 0 0
#  0 0 1 0
#  0 0 0 1
#  1 0 0 1
#  Each LED blinking .-.. twice.


def talker():

    # Send one message, or keep repeating?
    oneShot = False
    
    morseDi    	    = 0.2
    morsePause 	    = 0.1
    morseDa    	    = 0.6
    morseLongPause  = 1.0
    
    pub = rospy.Publisher('leds', LEDControl)
    rospy.init_node('ledControlTester', anonymous=True)

    msg1 = LEDControl(switch_array=[1,0,0,0])
    msg2 = LEDControl(switch_array=[0,1,0,0])
    msg3 = LEDControl(switch_array=[0,0,1,0])
    msg4 = LEDControl(switch_array=[0,0,0,1])
    msg5 = LEDControl(switch_array=[1,0,0,-1])    


    msg6 = LEDControl(switch_array=[-2,-2,-2,-2],
		      timed_switch_array=[TimedSwitch(switch_mode=-1,
						      num_cycles = 2,
						      pulse_pattern=[morseDi,
								     morsePause,
								     morseDa,
								     morsePause,
								     morseDi,
								     morsePause,
								     morseDi,
								     morseLongPause]),
					  TimedSwitch(switch_mode=-1,
						      num_cycles = 2,
						      pulse_pattern=[morseDi,
								     morsePause,
								     morseDa,
								     morsePause,
								     morseDi,
								     morsePause,
								     morseDi,
								     morseLongPause]),
					  TimedSwitch(switch_mode=-1,
						      num_cycles = 2,
						      pulse_pattern=[morseDi,
								     morsePause,
								     morseDa,
								     morsePause,
								     morseDi,
								     morsePause,
								     morseDi,
								     morseLongPause]),
					  TimedSwitch(switch_mode=-1,
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

      if msg1 is not None:
        rospy.loginfo("Msg1: " + str(msg1))
        pub.publish(msg1)
	rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

      if msg2 is not None:
	rospy.loginfo("Msg2: " + str(msg2))
	pub.publish(msg2)
	rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

      if msg3 is not None:
	rospy.loginfo("Msg3: " + str(msg3))
	pub.publish(msg3)
	rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

      if msg4 is not None:
	rospy.loginfo("Msg4: " + str(msg4))
	pub.publish(msg4)
	rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

      if msg5 is not None:
	rospy.loginfo("Msg5: " + str(msg5))
	pub.publish(msg5)
	rospy.sleep(INTER_PATTERN_SLEEP_DURATION)

      if msg6 is not None:
	rospy.loginfo("Msg6: " + str(msg6))
	pub.publish(msg6)
	rospy.sleep(INTER_PATTERN_SLEEP_DURATION)




if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
