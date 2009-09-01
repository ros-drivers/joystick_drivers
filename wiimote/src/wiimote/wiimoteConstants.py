################################################################################
#
# File:         wiimoteConstants.py
# RCS:          $Header: $
# Description:  Constants for Wii Arm Control
# Author:       Andreas Paepcke
# Created:      Thu Aug 13 11:44:04 2009
# Modified:     Wed Aug 19 08:13:40 2009 (Andreas Paepcke) paepcke@anw.willowgarage.com
# Language:     Python
# Package:      N/A
# Status:       Experimental (Do Not Distribute)
#
# #
################################################################################


_DEBUGLEVEL = 1
_MONITOR_LEVEL = 1

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


IR1       = 'ir1'
IR2       = 'ir2'
IR3       = 'ir3'

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

X_COORD   = 'x'
Y_COORD   = 'y'
Z_COORD   = 'z'
NORM_X    = 'normX'
NORM_Y    = 'normY'
NORM_Z    = 'normZ'

IR1       = 'ir1'
IR2       = 'ir2'
IR3       = 'ir3'
IR4       = 'ir4'

LED1_ON = 0x01
LED2_ON = 0x02
LED3_ON = 0x04
LED4_ON = 0x08

BATTERY_MAX = 0xD0                  # cwiid.CWIID_BATTERY_MAX


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

