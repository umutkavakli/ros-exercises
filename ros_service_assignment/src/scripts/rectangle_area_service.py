#! /usr/bin/env python

import rospy
from ros_service_assignment.srv import RectangleAreaService, RectangleAreaServiceRequest, RectangleAreaServiceResponse

def callback_rectangle_area(request):
    print("The Area\n%s * %s = %s" % (request.width, request.height, (request.width * request.height)))
    return RectangleAreaServiceResponse(request.width * request.height)

def rectangle_area_server():
    rospy.init_node('rectangle_area_server')
    serv = rospy.Service('/rectangle_area', RectangleAreaService, callback_rectangle_area)
    print("Ready to find area of a rectangle")
    rospy.spin()

if __name__ == "__main__":
    rectangle_area_server()