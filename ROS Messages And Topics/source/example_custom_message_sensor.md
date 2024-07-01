## Creating custom messages
#### 1- Define a custom message :
Let's say you want to create a custom message type for a sensor that measures temperature and humidity. 
Create a file named [SensorData.msg](../ros_topic_and_messages_pkg/msg/SensorData.msg) in your ROS package's [msg](../ros_topic_and_messages_pkg/msg) directory:

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
  SensorData.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
```
### 3-Write publisher and subscriber nodes
[publish_sensor_data](../ros_topic_and_messages_pkg/script/publish_sensor_data.py):

```py
#!/usr/bin/env python3

import rospy
from name_of_pkg.msg import SensorData
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

```
[subscribe_sensor_data.py](../ros_topic_and_messages_pkg/script/subscribe_sensor_data.py):


```py
#!/usr/bin/env python3

import rospy
from name_of_pkg.msg import SensorData

def sensor_data_callback(data):
    rospy.loginfo("Received sensor data: Temperature=%f, Humidity=%f", data.temperature, data.humidity)

def subscribe_sensor_data():
    rospy.init_node('sensor_subscriber', anonymous=True)
    rospy.Subscriber('sensor_topic', SensorData, sensor_data_callback)
    rospy.spin()

if __name__ == '__main__':
    subscribe_sensor_data()

```
![alt text](../images/5.png)

## [<-Back to main](../ros_topic_and_messages.md)

