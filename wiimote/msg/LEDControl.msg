# Message to request Wiimote LED operations. The Wiimote has four
# LEDs. This message type offers two methods for operating these
# indicators. The first method, 
#      o switch_array is an array of -1s, 0s, and 1s
#        -1 leaves the respective LED in its current state.
#         0 turns the respective LED off.
#         1 turns the respective LED on.
#
# The second is 
#      o timed_switch_array, an array of TimedSwitch. 
#        Each of these TimedSwitch instructions allow 
#        on/off sequences to be defined for each LED. 
#        See TimedSwitch.msg for details.

int8[4] switch_array
TimedSwitch[] timed_switch_array  
