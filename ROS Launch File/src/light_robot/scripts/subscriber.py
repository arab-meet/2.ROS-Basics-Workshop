#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def listner():
    rospy.init_node("stringSubscriber_node")
    rospy.Subscriber("stringTopic",String,callback)
    rospy.spin()

def callback(msgdata):
    rospy.loginfo(msgdata.data)    




if __name__=='__main__':
    listner()