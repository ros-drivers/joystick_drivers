################################################################################
#
# File:         wiiutils.py
# RCS:          $Header: $
# Description:  Various handy utilities
# Author:       Andreas Paepcke
# Created:      Thu Aug 13 15:11:56 2009
# Modified:     Thu Aug 13 15:22:45 2009 (Andreas Paepcke) paepcke@anw.willowgarage.com
# Language:     Python
# Package:      N/A
# Status:       Experimental (Do Not Distribute)
#
#
################################################################################

import sys
import time
import wiimoteConstants as acConst


#;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
#
#    Functions
#
#;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


def report(str, debuglevel=acConst._DEBUGLEVEL):
  """For error reporting, controlled by debuglevel."""
  if debuglevel > 0:
    print >> sys.stderr, str
  return

def log(str, file=None):
    if acConst._MONITOR_LEVEL > 0:
        print >> sys.stderr, str
        

def promptUsr(str):
  """Prompting user."""
  print >> sys.stderr, str
  return

def getTimeStamp():
  """Return current time as float of seconds since beginning of Epoch."""
  return time.time()

