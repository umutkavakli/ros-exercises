#! /usr/bin/env python

import rospy
from my_package.msg import IoTSensor

def iot_sensor_callback(data):
    rospy.loginfo("New IoT data received: (%d, %s, %.2f, %.2f)\n", data.id, data.name, data.temperature, data.humidity)

rospy.init_node('iot_sensor_subscriber_node', anonymous=True)
rospy.Subscriber('/iot_sensor_topic', IoTSensor, iot_sensor_callback)
rospy.spin()
