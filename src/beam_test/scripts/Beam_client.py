#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from beam_test.srv import *

def Beam_client(x):
    rospy.wait_for_service('Beam')
    try:
        Beams = rospy.ServiceProxy('Beam', Beam)
        resp1 = Beams(x)
        return resp1.ANG
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [ang]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "ang":
        x = sys.argv[1]
        
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s"%(x))
    print("%s = %s"%(x, Beam_client(x)))