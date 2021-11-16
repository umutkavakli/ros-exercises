#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
    rospy.init_node('move_node', anonymous=True)
    twist = Twist()
    twist.linear.x = 1.0
    twist.angular.z = 0.5

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
	    move()
    except rospy.ROSInterruptException:
        pass
