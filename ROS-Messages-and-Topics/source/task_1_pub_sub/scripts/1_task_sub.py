#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16

def number_callback(number):
    rospy.loginfo("the number received is : "+ str(number.data))

def sub_node():
    
    rospy.init_node('sub_node', anonymous=True)
    rospy.Subscriber('number_topic', Int16, number_callback)
    
    rospy.spin()
    
if __name__=='__main__':
    sub_node()