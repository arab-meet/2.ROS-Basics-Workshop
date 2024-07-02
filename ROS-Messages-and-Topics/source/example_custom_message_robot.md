## Creating custom messages
#### 1- Define a custom message :
Let's say you want to create a custom message type for a sensor that measures temperature and humidity. 
Create a file named [Position.msg](../ros_topic_and_messages_pkg/msg/Position.msg) in your ROS package's [msg](../ros_topic_and_messages_pkg/msg) directory:

```bash
# SensorData.msg
float32 temperature
float32 humidity
```
### 2- Compile the message
Make sure your `CMakeLists.txt` contains the following lines to ensure that your custom message is compiled:

```Cpp
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)
add_message_files(
  FILES
  Position.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
```
### 3-Write publisher and subscriber nodes
[publish_robot ](../ros_topic_and_messages_pkg/script/publish_robot.py):

```py
#!/usr/bin/env python3

import rospy
from name_of_pkg.msg import Position
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


```
[subscribe_robot ](../ros_topic_and_messages_pkg/script/subscribe_robot.py):


```py
#!/usr/bin/env python3

import rospy
from name_of_pkg.msg import Position

def callback(data):
    rospy.loginfo("Received: x=%f, y=%f, theta=%f", data.x, data.y, data.theta)

def subscriber():
    rospy.init_node('position_subscriber', anonymous=True)
    rospy.Subscriber('robot_position', Position, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()


```
![alt text](../images/7.png)

## [<-Back to main](../ros_topic_and_messages.md)

