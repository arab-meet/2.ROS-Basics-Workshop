## Creating Publisher and Subscriber

#### **Create a ROS Package**

    First, create a new ROS package if you don't already have one. You can do this using the`catkin_create_pkg` command

```bash
cd ~/catkin_ws/src
catkin_create_pkg chatter_pkg std_msgs rospy roscpp
```

#### [Publisher](../ros_topic_and_messages_pkg/script/publish.py):

#### Python Publisher

#### **Create the Publisher Script**

Navigate to the `src` directory of your package and create a Python script named `talker.py`

```bash
cd chatter_pkg/src
touch talker.py
```

#### **Edit the Publisher Script**

Open `talker.py` in your favorite text editor and add the following code

```py

import rospy  # Importing the rospy package, which is the Python client library for ROS.
from std_msgs.msg import String  # Importing the String message type from the std_msgs package.

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)  # Creates a publisher object named pub that publishes messages of type String to the topic named 'chatter'.
    rospy.init_node('talker', anonymous=True)  # Initializes a ROS node named 'talker'. The anonymous=True argument ensures that if another node with the same name is launched, the node name will be modified to make it unique.
    rate = rospy.Rate(10)  # Creates a Rate object that controls the loop rate of the while loop to 10 times per second (10 Hz).
    while not rospy.is_shutdown():  # The while loop runs until the ROS node is shutdown.
        hello_str = "hello world"  # Creates a string message "hello world".
        rospy.loginfo(hello_str)  # Logs the message to the ROS console at INFO level.
        pub.publish(hello_str)  # Publishes the message to the topic 'chatter'.
        rate.sleep()  # Sleeps to maintain the specified loop rate.
  
if __name__ == '__main__':  # Checks if the script is being run directly.
    try:
        talker()  # Calls the talker() function.
    except rospy.ROSInterruptException:  # Catches the ROSInterruptException exception, which is raised when the node is interrupted or shutdown.
        pass  # Passes without doing anything, ensuring the script exits gracefully without throwing an error when it's terminated.
```

#### **Make the Script Executable**

Make sure the `talker.py` script is executable

```bash
chmod +x talker.py
```

---

#### C++ Publisher

1. **Create the Publisher Script**

Navigate to the `src` directory of your package and create a C++ script named `talker.cpp`:

```bash
touch talker.cpp
```

Open `talker.cpp` in your favorite text editor and add the following code

```cpp
#include "ros/ros.h"                 // Include the main ROS library
#include "std_msgs/String.h"         // Include the standard String message type from ROS

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");   // Initialize the ROS system and name the node "talker"
  ros::NodeHandle n;                 // Create a node handle to communicate with ROS system

  // Create a publisher object that publishes messages of type std_msgs::String on the "chatter" topic
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);           // Set the loop frequency to 10 Hz

  int count = 0;                     // Initialize a counter for the messages
  while (ros::ok())                  // Keep looping while ROS is running
  {
    std_msgs::String msg;            // Create a new String message object
    std::stringstream ss;            // Create a string stream to construct the message data
    ss << "hello world " << count;   // Write the message content to the string stream
    msg.data = ss.str();             // Set the message data to the content of the string stream

    ROS_INFO("%s", msg.data.c_str()); // Print the message content to the console

    chatter_pub.publish(msg);        // Publish the message

    ros::spinOnce();                 // Handle callbacks (not needed here, but good practice)
    loop_rate.sleep();               // Sleep for the remainder of the loop cycle time
    ++count;                         // Increment the message counter
  }
  return 0;
}
```

3. **Edit the `CMakeLists.txt` File**

Add the following lines to your `CMakeLists.txt` file to compile the C++ node

```cmake
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
```

---

#### [Subscriber](../ros_topic_and_messages_pkg/script/subscribe.py)

#### Python Subscriber

#### **Create the Subscriber Script**

Navigate to the `src` directory of your package and create a Python script named `listener.py`

```bash
touch listener.py
```

#### **Edit the Subscriber Script**

Open `listener.py` in your favorite text editor and add the following code

```py
#!/usr/bin/env python3  # Specifies the interpreter to use when executing the script (Python 3).

import rospy  
from std_msgs.msg import String 

def listener():
    rospy.init_node('listener', anonymous=True)  # Initializes a ROS node named 'listener'. The anonymous=True argument ensures that each instance of the node has a unique name.
    rospy.Subscriber('chatter', String, callback)  # Creates a subscriber that listens to messages on the 'chatter' topic. It specifies that the messages will be of type String and that the callback function to be called when a message is received is callback.
    rospy.spin()  # Keeps the node running until it is explicitly stopped.

def callback(data):
    rospy.loginfo('I heard %s' % data.data)  # Logs the received message to the ROS logging system. It prints the received message to the console with the prefix 'I heard'.

if __name__ == '__main__':  # Checks if the script is being run directly.
    listener()  # Calls the listener function to start the ROS node.
```

#### **Make the Script Executable**

Make sure the `listener.py` script is executable

```bash
chmod +x listener.py
```

---



#### C++ Subscriber

1. **Create the Subscriber Script**

Navigate to the `src` directory of your package and create a C++ script named `listener.cpp`:

```bash
touch listener.cpp
```

2. **Edit the Subscriber Script**

Open `listener.cpp` in your favorite text editor and add the following code:

```cpp
#include "ros/ros.h"                 // Include the main ROS library
#include "std_msgs/String.h"         // Include the standard String message type from ROS

// Callback function to handle incoming messages
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  // Print the received message content to the console
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener"); // Initialize the ROS system and name the node "listener"
  ros::NodeHandle n;                 // Create a node handle to communicate with ROS system

  // Create a subscriber object that listens to messages of type std_msgs::String on the "chatter" topic
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();                       // Enter a loop and process callbacks (i.e., call chatterCallback when a message is received)
  return 0;
}

```


3. **Edit the `CMakeLists.txt` File**

Add the following lines to your `CMakeLists.txt` file to compile the C++ node:

```cmake
add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
```

#### **Build the Package**

Navigate to your catkin workspace and build your package

```bash
cd ../../..
catkin_make
source devel/setup.bash
```

#### **Run the Publisher and subscriber Node**

open terminal to run `roscore`

```bash
roscore
```

Open a new terminal to run  your publisher node using the following command

```bash
rosrun chatter_pkg talker.py
```

Or your C++ publisher node:

```bash
rosrun chatter_pkg talker
```

Open a new terminal to run your subscriber node

```bash
rosrun chatter_pkg listener.py
```

Or your C++ subscriber node:

```bash
rosrun chatter_pkg listener
```

---



![alt text](../images/6.png)

## [&lt;-Back to main](../ros_topic_and_messages.md)
