#!/usr/bin/env python3

import rospy
from custom_msg_task_pkg.msg import my_info

def info_data_callback(data):
        rospy.loginfo("Received info \n name: " + data.name + "\n age is: " + str(data.age) + "\n degree is " + str(data.high_school_degree))

def subscribe_sensor_data():
    rospy.init_node('info_data_subscriber_node', anonymous=True)
    rospy.Subscriber('info_topic', my_info, info_data_callback)
    rospy.spin()

if __name__ == '__main__':
    subscribe_sensor_data()
