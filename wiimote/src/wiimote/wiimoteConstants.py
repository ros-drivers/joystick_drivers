################################################################################
#
# File:         wiimoteConstants.py
# RCS:          $Header: $
# Description:  Constants for Wii Arm Control
# Author:       Andreas Paepcke
# Created:      Thu Aug 13 11:44:04 2009
# Modified:     Mon Nov  9 12:43:53 2009 (Andreas Paepcke) paepcke@anw.willowgarage.com
# Language:     Python
# Package:      N/A
# Status:       Experimental (Do Not Distribute)
#
# #
################################################################################


_DEBUGLEVEL = 1
_MONITOR_LEVEL = 1

# TimedSwitch message field value constants:
SWITCH_ON  = 1
SWITCH_OFF = 0
SWITCH_PULSE_PATTERN = -1

# Output indicators on the Wiimote:

RUMBLE = 1
LED    = 2

# Buttons in cwiid structure:

BTN_1       = 0x0002                  # cwiid.CWIID_BTN_1
BTN_2       = 0x0001                  # cwiid.CWIID_BTN_2
BTN_B       = 0x0004                  # cwiid.CWIID_BTN_B
BTN_A       = 0x0008                  # cwiid.CWIID_BTN_A
BTN_MINUS = 0x0010                  # cwiid.CWIID_BTN_MINUS
BTN_PLUS  = 0x1000                  # cwiid.CWIID_BTN_PLUS
BTN_LEFT  = 0x0100                  # cwiid.CWIID_BTN_LEFT
BTN_RIGHT = 0x0200                  # cwiid.CWIID_BTN_RIGHT
BTN_DOWN  = 0x0400                  # cwiid.CWIID_BTN_DOWN
BTN_UP    = 0x0800                  # cwiid.CWIID_BTN_UP
BTN_HOME  = 0x0080                  # cwiid.CWIID_BTN_HOME

X   = 0
Y   = 1
Z   = 2

PHI     = 0
THETA   = 1
PSI     = 2


# Buttons order in Wiimote ROS message array:

MSG_BTN_1     = 0
MSG_BTN_2     = 1
MSG_BTN_A     = 2
MSG_BTN_B     = 3
MSG_BTN_PLUS  = 4
MSG_BTN_MINUS = 5
MSG_BTN_LEFT  = 6
MSG_BTN_RIGHT = 7
MSG_BTN_UP    = 8
MSG_BTN_DOWN  = 9
MSG_BTN_HOME  = 10

X_COORD   = 'x'
Y_COORD   = 'y'
Z_COORD   = 'z'
NORM_X    = 'normX'
NORM_Y    = 'normY'
NORM_Z    = 'normZ'

NUM_IR_SENSORS = 4
IR1       = 'ir1'
IR2       = 'ir2'
IR3       = 'ir3'
IR4       = 'ir4'

NUM_LEDS = 4
LED1_ON = 0x01
LED2_ON = 0x02
LED3_ON = 0x04
LED4_ON = 0x08

# Indices into a two-tuple of info about battery state
# in wiimote messages:
#   0: Percentage of battery left.
#   1: The raw battery reading

BATTERY_PERCENTAGE = 0
BATTERY_RAW = 1

# Turning wiimote accelerator readings from g's to m/sec^2:
EARTH_GRAVITY = 9.80665             # m/sec^2

# Turning wiimote gyro readings to radians/sec.
# This scale factor is highly approximate. Procedure:
#    - Tape Wiimote to center of an office chair seat
#    - Rotate the chair at approximately constant speed
#      for 10 seconds. This resulted in 6 chair revolutions
#    - On average, the Wiimote gyro read 3570 during this
#      experiment. 
#    - Speed of chair revolving: 
#         * One full circle is: 2#pi radians
#         * Six revolutions = 12pi radians. ==> 12pi rad in 10 sec ==> 1.2pi rad/sec
#         * => 3570 == 1.2pi
#         * => x*3570 = 1.2pi
#         * => x = 1.2pi/3570 (1.2pi = 3.769908)
#         * => scale factor = 0.001055997
# So multiplying the gyro readings by this factor
# calibrates the readings to show angular velocity
# in radians/sec.
GYRO_SCALE_FACTOR = 0.001055997

# Status type of message from Wii to us:
WII_MSG_TYPE_STATUS      = 0
WII_MSG_TYPE_BTN         = 1
WII_MSG_TYPE_ACC         = 2
WII_MSG_TYPE_IR          = 3
# WII_MSG_TYPE_NUNCHUK     = 4
# WII_MSG_TYPE_CLASSIC     = 5
# WII_MSG_TYPE_BALANCE     = 6
WII_MSG_TYPE_MOTIONPLUS  = 7 # Gyro
WII_MSG_TYPE_ERROR       = 8
WII_MSG_TYPE_UNKNOWN     = 9


#define CWIID_IR_X_MAX        1024
#define CWIID_IR_Y_MAX        768

