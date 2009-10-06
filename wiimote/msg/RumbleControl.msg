# Message to control the Wiimote rumble (vibrator).
# We simply use a TimedSwitch. So rumble can
# be turned on (switch_mode == 1), off (switch_mode == 0),
# or pulsed with a timing pattern (switch_mode == -1).
# Number of times the cycle is repeated: num_cycles
# (-1: repeat till next RumbleControl message). pulse_pattern
# contains the time on/off pattern. (see also TimedSwitch.msg)

TimedSwitch rumble

