# Message to request Wiimote LED operations. The Wiimote has four
# LEDs. This message type offers two methods for operating these
# indicators. The first method turns each of the LEDs on or off:
#      o switch_array is an array of -1s, 0s, and 1s
#        -1 leaves the respective LED in its current state.
#         0 turns the respective LED off.
#         1 turns the respective LED on.
#
# The second method is to define a blink pattern for each of
# the LEDs. Accomplished as follows:
#      o switch_array[0] must be set to -2. The other
#        switch_array entries don't matter.
#      o timed_switch_array is an array of TimedSwitch;
#        one TimedSwitch for each LED. The TimedSwitch
#        entries allow callers to define a blink pattern
#        for each LED.
#        See TimedSwitch.msg for details.

int8[4] switch_array
TimedSwitch[] timed_switch_array  
