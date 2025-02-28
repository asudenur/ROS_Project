#!/usr/bin/env python3

from ros_tutorials.srv import RectangleArea, RectangleAreaRequest, RectangleAreaResponse
import rospy

def handle_rectangle_area(req):
    # Alanı hesaplama (genişlik * yükseklik)
    area = req.width * req.height
    print("Returning area of rectangle with width [%s] and height [%s]: %s" % (req.width, req.height, area))
    return RectangleAreaResponse(area)

def rectangle_area_server():
    rospy.init_node('rectangle_area_server')
    # Servis oluşturma
    s = rospy.Service('rectangle_area', RectangleArea, handle_rectangle_area)
    print("Ready to calculate rectangle area.")
    rospy.spin()

if __name__ == "__main__":
    rectangle_area_server()

