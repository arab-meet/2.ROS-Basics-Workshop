#!/usr/bin/env python3

import rospy
from ros_topic_and_messages_pkg.msg import Position
from random import random

def publisher():
    rospy.init_node('position_publisher', anonymous=True)
    pub = rospy.Publisher('robot_position', Position, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        msg = Position()
        msg.x = random() * 10  # Random x position
        msg.y = random() * 10  # Random y position
        msg.theta = random() * 3.14  # Random theta (orientation)
        rospy.loginfo("Publishing: x=%f, y=%f, theta=%f", msg.x, msg.y, msg.theta)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
