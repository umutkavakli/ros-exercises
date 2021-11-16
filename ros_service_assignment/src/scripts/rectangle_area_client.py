#! /usr/bin/env python

import sys
import rospy
from ros_service_assignment.srv import RectangleAreaService, RectangleAreaServiceRequest, RectangleAreaServiceResponse

def calculate_area(width, height):
    rospy.wait_for_service('rectangle_area')
    try:
        rectangle_area = rospy.ServiceProxy('/rectangle_area', RectangleAreaService)
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
    print("Requesting area of width %s and height %s" % (width, height))
    area = calculate_area(width, height)
    print("The area of %s and %s is %s" % (width, height, area))