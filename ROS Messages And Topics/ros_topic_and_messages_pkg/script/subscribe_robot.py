#!/usr/bin/env python3

import rospy
from ros_topic_and_messages_pkg.msg import Position

def callback(data):
    rospy.loginfo("Received: x=%f, y=%f, theta=%f", data.x, data.y, data.theta)

def subscriber():
    rospy.init_node('position_subscriber', anonymous=True)
    rospy.Subscriber('robot_position', Position, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
