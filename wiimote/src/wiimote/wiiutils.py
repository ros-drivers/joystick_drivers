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

from __future__ import print_function
from __future__ import absolute_import
import sys
import time
from . import wiimoteConstants as acConst
from math import sqrt
import numpy as np
import random


def report(str, debuglevel=acConst._DEBUGLEVEL):
    """For error reporting, controlled by debuglevel."""
    if debuglevel > 0:
        print(str, file=sys.stderr)
    return


def log(str, file=None):
    if acConst._MONITOR_LEVEL > 0:
        print(str, file=sys.stderr)


def promptUsr(str):
    """Prompting user."""
    print(str, file=sys.stderr)
    return


def getTimeStamp():
    """Return current time as float of seconds since beginning of Epoch."""
    return time.time()


if __name__ == '__main__':
    foo = np.array([(4.0, 3.0, 3.0)])
    bar = np.array([(4.0, 3.0, 3.0)])

    isGreater = np.greater(foo, bar)
    isBad = np.greater(foo, bar).any()
    test = (foo > bar).any()

    print(repr(isGreater))
    print(isBad)
    print(test)
