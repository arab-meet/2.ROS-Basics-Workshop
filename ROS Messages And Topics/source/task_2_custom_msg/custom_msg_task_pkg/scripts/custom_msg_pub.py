#!/usr/bin/env python3

import rospy
from custom_msg_task_pkg.msg import my_info

def publish_sensor_data():
    rospy.init_node('my_info_publisher', anonymous=True)
    pub = rospy.Publisher('info_topic', my_info, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    info_data = my_info()
    while not rospy.is_shutdown():
        info_data.name = "Mo3taz"
        info_data.age = 24
        info_data.high_school_degree = 92
        
        rospy.loginfo("Publishing info\n name: " + info_data.name + "\n age is: " + str(info_data.age) + "\n degree is " + str(info_data.high_school_degree))
        pub.publish(info_data)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_sensor_data()
    except rospy.ROSInterruptException:
        pass