# TimedSwitch allows sender to:
#    o turn a switch on,
#    o turn a switch off, and
#    o repeat an on/off pattern forever or for a
#          given number of times.
# Fields:
#     o switch_mode:
#         -1: turn on
#          0: turn off
#        n>0: run pulse pattern till turnoff message is received
#
#     o pulse_pattern:
#          A series of time durations in fractions of a second. The
#          first number is the duration for having the switch on.
#          The second number is the duration for which the switch
#          is off. The third is an 'on' period again, etc. up to 10.
#          A pattern is terminated with a 0 or with the specification
#          of the 10th duration.
#           
#          Example: [1,1,0] specifies an on-off sequence of 1 second.               

int32 switch_mode
float32[10] pulse_pattern
