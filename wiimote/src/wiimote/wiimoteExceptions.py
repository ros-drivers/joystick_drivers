################################################################################
#
# File:         wiimoteExceptions.py
# RCS:          $Header: $
# Description:  Exception Classes for Wiimote Controller
# Author:       Andreas Paepcke
# Created:      Thu Aug 13 09:01:17 2009
# Modified:     Mon Aug 17 11:27:02 2009 (Andreas Paepcke) paepcke@anw.willowgarage.com
# Language:     Python
# Package:      N/A
# Status:       Experimental (Do Not Distribute)
#
# 
#
################################################################################

class WiimoteError(Exception):
  """Mother of all Wiimote exceptions"""

  errMsg = None

  def __init__(self, theErrMsg):
    self.errMsg = theErrMsg
  
  def __str__(self):
    return self.errMsg


class WiimoteNotFoundError(WiimoteError):
  """Tried to pair but failed."""


class WiimoteEnableError(WiimoteError):
  """Found wiimote, but couldn't enable it."""


class CallbackStackMultInstError(WiimoteError):
  """Code attempted to create a second callback stack instance."""

class ResumeNonPausedError(WiimoteError):
  """Code attempted to resume callbacks without first pausing."""

class CallbackStackEmptyError(WiimoteError):
  """Attemp to operate on an empty callback stack."""
  
