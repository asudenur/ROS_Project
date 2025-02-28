#!/usr/bin/env python3

from ros_tutorials.srv import DecTwoInts
from ros_tutorials.srv import DecTwoIntsRequest
from ros_tutorials.srv import DecTwoIntsResponse

import rospy

def handle_dec_two_ints(req):
    print ("Returning [%s - %s = %s]"%(req.a, req.b, (req.a - req.b)))
    return DecTwoIntsResponse(req.a - req.b)

def dec_two_ints_server():
    rospy.init_node('dec_two_ints_server')
    s = rospy.Service('dec_two_ints', DecTwoInts, handle_dec_two_ints)
    print ("Ready to dec two ints.")
    rospy.spin()
    
if __name__ == "__main__":
    dec_two_ints_server()
