#!/usr/bin/env
from __future__ import print_function
from bluetooth import *

print("Looking for compatible devices")

nearby_devices = discover_devices(lookup_names = 1,duration = 10,flush_cache=True)

for addr,name in nearby_devices:

     if name == 'Nintendo RVL-CNT-01' or name == 'Nintendo RVL-WBC-01':
          print ('Compatible controller:\n %s - %s' % (addr, name))
     if name == 'Nintendo RVL-CNT-01-TR':
          print ("Incompatible controller:\n %s - %s" % (addr,name))

