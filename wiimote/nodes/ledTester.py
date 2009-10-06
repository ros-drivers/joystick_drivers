#!/usr/bin/env python
import roslib; roslib.load_manifest('wiimote')
import rospy
from wiimote.msg import LEDControl
from wiimote.msg import TimedSwitch

def talker():

    # Send one message, or keep repeating?
    oneShot = False
    
    morseDi    	    = 0.2
    morsePause 	    = 0.1
    morseDa    	    = 0.6
    morseLongPause  = 1.0
    
    pub = rospy.Publisher('leds', LEDControl)
    rospy.init_node('ledControlTester', anonymous=True)

    msg1 = None
    msg2 = None

#    msg1 = LEDControl(switch_array=[1,0,0,0])
#    msg2 = LEDControl(switch_array=[0,0,0,0])


#     msg1 = LEDControl(switch_array=[-1,-1,-1,-1],
#                       #switch_array=[1,0,1,0],
# 		      timed_switch_array=[TimedSwitch(switch_mode=-1,
# 						      num_cycles = 2,
# 						      pulse_pattern=[morseDi,
# 								     morsePause,
# 								     morseDa,
# 								     morsePause,
# 								     morseDi,
# 								     morsePause,
# 								     morseDi,
#								     morseLongPause])])


    msg2 = LEDControl(switch_array=[-1,-1,-1,-1],
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
	rospy.sleep(5.0)
      if msg2 is not None:
	rospy.loginfo("Msg2: " + str(msg2))
	pub.publish(msg2)
	rospy.sleep(5.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
