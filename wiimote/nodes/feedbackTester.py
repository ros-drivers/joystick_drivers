#!/usr/bin/env python

import roslib; roslib.load_manifest('wiimote')
import rospy
from sensor_msgs.msg import JoyFeedbackArray
from sensor_msgs.msg import JoyFeedback

# Test the wiimote package LED control. Expect the following LED
# patterns of INTER_PATTERN_SLEEP_DURATION seconds duration each:


INTER_PATTERN_SLEEP_DURATION = 0.2

def talker():    
    pub = rospy.Publisher('/joy/set_feedback', JoyFeedbackArray, queue_size=1)
    rospy.init_node('ledControlTester', anonymous=True)

    led0 = JoyFeedback()
    led0.type = JoyFeedback.TYPE_LED
    led0.id = 0
    led1 = JoyFeedback()
    led1.type = JoyFeedback.TYPE_LED
    led1.id = 1
    led2 = JoyFeedback()
    led2.type = JoyFeedback.TYPE_LED
    led2.id = 2
    led3 = JoyFeedback()
    led3.type = JoyFeedback.TYPE_LED
    led3.id = 3
    rum = JoyFeedback()
    rum.type = JoyFeedback.TYPE_RUMBLE
    rum.id = 0


    while not rospy.is_shutdown():
      
      msg = JoyFeedbackArray()
      msg.array = [led0, led1, led2, led3, rum]
      
      led0.intensity = 0.2
      led3.intensity = 0.2
      rum.intensity = 0.49

      if msg is not None:
          rospy.logdebug("Msg: " + str(msg))
          pub.publish(msg)
          rospy.sleep(INTER_PATTERN_SLEEP_DURATION)
          
      led0.intensity = 1.0
      rum.intensity = 0.51

      if msg is not None:
          rospy.logdebug("Msg: " + str(msg))
          pub.publish(msg)
          rospy.sleep(INTER_PATTERN_SLEEP_DURATION)
          
      led0.intensity = 0.0
      led1.intensity = 1.0
      rum.intensity = 0.0

      if msg is not None:
          rospy.logdebug("Msg: " + str(msg))
          pub.publish(msg)
          rospy.sleep(INTER_PATTERN_SLEEP_DURATION)
          
      led1.intensity = 0.0
      led2.intensity = 1.0
      rum.intensity = 0.7

      if msg is not None:
          rospy.logdebug("Msg: " + str(msg))
          pub.publish(msg)
          rospy.sleep(INTER_PATTERN_SLEEP_DURATION)
          
      led2.intensity = 0.0
      led3.intensity = 1.0
      rum.intensity = 0.49

      if msg is not None:
          rospy.logdebug("Msg: " + str(msg))
          pub.publish(msg)
          rospy.sleep(INTER_PATTERN_SLEEP_DURATION)
          
      led1.intensity = 1.0
      led2.intensity = 1.0
      rum.intensity = 1.0

      if msg is not None:
          rospy.logdebug("Msg: " + str(msg))
          pub.publish(msg)
          rospy.sleep(INTER_PATTERN_SLEEP_DURATION)
          
      led0.intensity = 1.0
      led1.intensity = 0.4
      led2.intensity = 0.4
      
      msg.array = [led0, led1, led2]

      if msg is not None:
          rospy.logdebug("Msg: " + str(msg))
          pub.publish(msg)
          rospy.sleep(INTER_PATTERN_SLEEP_DURATION)


if __name__ == '__main__':
    
  print("\n   ****************************************************************\n")
  print("****     You should see six LED on/off configurations, and feel Rumbles!  ****")
  print("\n   **************************************************************")
  print("[off, off, off, off]")
  print("[on,  off, off, off]")
  print("[off, on,  off, off]")
  print("[off, off, on,  off]")
  print("[off, off, off, on ]")
  print("[off, on,  on,  on ]")
  print("[on,  off, off, on ]")
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
