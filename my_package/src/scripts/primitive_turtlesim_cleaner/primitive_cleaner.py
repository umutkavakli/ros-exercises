#! /usr/bin/env python

from codecs import lookup
import rospy
import math
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

x = 0
y = 0
yaw = 0

def move(velocity_publisher, speed, distance, is_forward=True):
    velocity = Twist()

    global x, y
    x0 = x
    y0 = y

    if is_forward:
        velocity.linear.x = abs(speed)
    else:
        velocity.linear.x = -abs(speed)
    
    distance_moved = 0.0
    rate = rospy.Rate(100)

    while True:
        rospy.loginfo("Turtlesim moves")
        velocity_publisher.publish(velocity)
        rate.sleep()
        distance_moved = math.sqrt((x-x0)**2 + (y-y0)**2)
        print(distance_moved)
        if distance_moved > distance:
            rospy.loginfo("Reached")
            break
    
    velocity.linear.x = 0
    velocity_publisher.publish(velocity)

def rotate(velocity_publisher, angular_speed_degree, relative_speed_degree, clockwise=False):
    velocity = Twist()
    angular_speed = math.radians(abs(angular_speed_degree))

    if clockwise:
        velocity.angular.z = -abs(angular_speed)
    else:
        velocity.angular.z = abs(angular_speed)


    rate = rospy.Rate(100)
    t0 = rospy.Time.now().to_sec()

    while True:
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        rate.sleep()

        if current_angle_degree > relative_speed_degree:
            rospy.loginfo("Reached")
            break

    velocity.angular.z = 0
    velocity_publisher.publish(velocity)

def go_to_goal(velocity_publisher, x_goal, y_goal): 
    global x, y, yaw
    velocity = Twist()
    rate = rospy.Rate(100)

    while True:
        K_linear = 0.5
        distance = math.sqrt(((x_goal-x)**2) + ((y_goal-y)**2))
        linear_speed = distance * K_linear

        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-yaw)*K_angular

        velocity.linear.x = linear_speed
        velocity.angular.z = angular_speed

        velocity_publisher.publish(velocity)
        rate.sleep()

        print('x = ', x, ', y = ', y, ', distance to goal: ', distance)

        if distance < 0.01:
            rospy.loginfo("Reached")
            break

def setDesiredOrientation(publisher, speed_in_degree, desired_angle_degree):
    relative_angle_radians = math.radians(desired_angle_degree) - yaw
    clockwise = 0

    if relative_angle_radians < 0:
        clockwise = 1
    
    print("Relative_angle_radians: ", math.degrees(relative_angle_radians))
    print("desired_angle_degree: ", desired_angle_degree)
    rotate(publisher, speed_in_degree, math.degrees(abs(relative_angle_radians)), clockwise)

def spiral(velocity_publisher, rk, wk):
    velocity = Twist()    
    rate = rospy.Rate(1)

    while (x < 10.5) and (y < 10.5):
        rk += 0.5
        velocity.linear.x = rk
        velocity.linear.y = 0
        velocity.linear.z = 0
        velocity.angular.x = 0
        velocity.angular.y = 0
        velocity.angular.z = wk

        velocity_publisher.publish(velocity)
        rate.sleep()
    
    velocity.linear.x = 0
    velocity.angular.z = 0
    velocity_publisher.publish(velocity)

def gridClean(publisher):
    desired_pose = Pose()
    desired_pose.x = 1
    desired_pose.y = 1
    desired_pose.theta = 0

    go_to_goal(publisher, 1, 1)

    setDesiredOrientation(publisher, 30, math.radians(desired_pose.theta))

    for i in range(4):
        move(publisher, 2.0, 1.0, True)
        rotate(publisher, 20, 90, False)
        move(publisher, 2.0, 9.0, True)
        rotate(publisher, 20, 90, True)
        move(publisher, 2.0, 1.0, True)
        rotate(publisher, 20, 90, True)
        move(publisher, 2.0, 9.0, True)
        rotate(publisher, 20, 90, False)
    pass

def poseCallback(pose_msg):
    global x, y, yaw
    x = pose_msg.x
    y = pose_msg.y
    yaw = pose_msg.theta

if __name__ == "__main__":
    try:
        rospy.init_node('turtlesim_motion_pose', anonymous=False)

        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        position_topic = '/turtle1/pose'
        sub = rospy.Subscriber(position_topic, Pose, poseCallback)
        time.sleep(2)

        #move(velocity_publisher, 5, 2, False)
        #rotate(velocity_publisher, 30, 90)
        #go_to_goal(velocity_publisher, 3.0, 5.0)
        #setDesiredOrientation(velocity_publisher, 30, 90)
        #spiral(velocity_publisher, 0, 4)
        gridClean(velocity_publisher)
         
    except rospy.ROSInternalException:
        rospy.loginfo("Node terminated.")