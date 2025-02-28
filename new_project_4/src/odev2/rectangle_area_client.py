#!/usr/bin/env python3

import sys
import rospy
from ros_tutorials.srv import RectangleArea, RectangleAreaRequest, RectangleAreaResponse

def rectangle_area_client(width, height):
    rospy.wait_for_service('rectangle_area')
    try:
        # Servis Proxy tanımlama
        rectangle_area = rospy.ServiceProxy('rectangle_area', RectangleArea)
        # Servis çağrısı yapma ve yanıt alma
        resp = rectangle_area(width, height)
        return resp.area
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    if len(sys.argv) == 3:
        width = float(sys.argv[1])
        height = float(sys.argv[2])
    else:
        print("%s [width height]" % sys.argv[0])
        sys.exit(1)

    print("Requesting area of rectangle with width %s and height %s" % (width, height))
    area = rectangle_area_client(width, height)
    print("The area of the rectangle is:[%s] * [%s] = %s" %(width, height, area))
    
