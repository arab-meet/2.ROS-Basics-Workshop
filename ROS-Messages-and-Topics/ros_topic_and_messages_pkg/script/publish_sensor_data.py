#!/usr/bin/env python3

import rospy
from ros_topic_and_messages_pkg.msg import SensorData
import random

def publish_sensor_data():
    rospy.init_node('sensor_publisher', anonymous=True)
    pub = rospy.Publisher('sensor_topic', SensorData, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        sensor_data = SensorData()
        sensor_data.temperature = random.uniform(20.0, 30.0)
        sensor_data.humidity = random.uniform(40.0, 60.0)
        rospy.loginfo("Publishing sensor data: Temperature=%f, Humidity=%f", sensor_data.temperature, sensor_data.humidity)
        pub.publish(sensor_data)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_sensor_data()
    except rospy.ROSInterruptException:
        pass

