#!/usr/bin/env python3

import rospy
from ros_topic_and_messages_pkg.msg import SensorData

def sensor_data_callback(data):
    rospy.loginfo("Received sensor data: Temperature=%f, Humidity=%f", data.temperature, data.humidity)

def subscribe_sensor_data():
    rospy.init_node('sensor_subscriber', anonymous=True)
    rospy.Subscriber('sensor_topic', SensorData, sensor_data_callback)
    rospy.spin()

if __name__ == '__main__':
    subscribe_sensor_data()

