#!/usr/bin/env python3

import sys
import rospy
from ros_tutorials.srv import DecTwoInts
from ros_tutorials.srv import DecTwoIntsRequest
from ros_tutorials.srv import DecTwoIntsResponse

def dec_two_ints_client(x, y):
    rospy.wait_for_service('dec_two_ints')
    try:
        dec_two_ints = rospy.ServiceProxy('dec_two_ints', DecTwoInts)
        resp1 = dec_two_ints(x, y)
        return resp1.cikart
    except rospy.ServiceException(e):
        print ("Service call failed: %s"%e)

def usage():
    return 

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print ("%s [x y]"%sys.argv[0])
        sys.exit(1)
    print ("Requesting %s+%s"%(x, y))
    s = dec_two_ints_client(x, y)
    print ("%s + %s = %s"%(x, y, s))
