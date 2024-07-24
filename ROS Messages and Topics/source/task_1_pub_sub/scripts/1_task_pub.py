#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int16

def pub_node():
    pub_number = rospy.Publisher('number_topic', Int16, queue_size=10)
    rospy.init_node('pub_node', anonymous=True)
    rate = rospy.Rate(1) 
    number = 1
    while not rospy.is_shutdown():
        pub_number.publish(number)
        rospy.loginfo("the number published now is : "+ str(number))
        number += 1
        rate.sleep()
        
if __name__=='__main__':
    try:
        pub_node()
    except rospy.ROSlnterruptException:
        pass