#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()
    
def callback(data):
    rospy.loginfo('l heard %s' % data.data )

if __name__=='__main__':
 listener()
