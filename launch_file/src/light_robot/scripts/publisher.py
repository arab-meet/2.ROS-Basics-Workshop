#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    rospy.init_node("stringPublisher_node")
    pub = rospy.Publisher("stringTopic",String,queue_size=1)
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        text_str="I am Learning Robotics and ROS"
        rospy.loginfo(text_str)
        pub.publish(text_str)
        rate.sleep()
        





if __name__=='__main__' :
    try:
        talker()
    except rospy.ROSInterruptException :
        pass
    
