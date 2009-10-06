# TimedSwitch allows sender to:
#    o turn a switch on,
#    o turn a switch off, and
#    o repeat an on/off pattern forever or for a
#          given number of times.
# Fields:
#     o switch_mode:
#          1: turn on  (num_cycles and pulse_pattern fields are ignored)
#          0: turn off (num_cycles and pulse_pattern fields are ignored)
#         -1: repeat an on/off pattern for as long
#             as is indicated in the num_cycles field. The
#             pattern is defined in the pulse_pattern field.
#
#     o num_cycles:
#          n>=0: run the pattern that is defined in pulse_pattern
#                n times.
#          n==-1: run the pattern that is defined in pulse_pattern
#                 until a new TimedSwitch message is sent.              
#
#     o pulse_pattern:
#          A series of time durations in fractions of a second. The
#          first number is the duration for having the switch on.
#          The second number is the duration for which the switch
#          is off. The third is an 'on' period again, etc.
#          A pattern is terminated with the end of the array.
#           
#          Example: [1,1] specifies an on-off sequence of 1 second.               

int8 switch_mode
int32 num_cycles
float32[] pulse_pattern
