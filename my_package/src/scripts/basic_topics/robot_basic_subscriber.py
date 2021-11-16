#! /usr/bin/env python

import rospy
from turtlesim.msg import Pose

def callback(msg):
    print('Pose of Robot\nx: {}\ny: {}\nyaw: {}\n'.format(msg.x, msg.y, msg.theta))


def robot_pose():
    rospy.init_node('pose_finder', anonymous=True)
    sub = rospy.Subscriber('/turtle1/pose', Pose, callback)
    rospy.spin()

if __name__ == '__main__':
    robot_pose()
    
