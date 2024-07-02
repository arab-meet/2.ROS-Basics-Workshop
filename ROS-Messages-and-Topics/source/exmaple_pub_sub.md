
## Creating Publisher and Subscriber
[Publisher](../ros_topic_and_messages_pkg/script/publish.py):

  

```py
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        hello_str = "hello world" 
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        
if __name__=='__main__':
    try:
        talker()
    except rospy.ROSlnterruptException:
        pass
```
- `#!/usr/bin/env python3`: This is a shebang line specifying the interpreter to use when executing the script (Python 3).

- `import rospy`: Importing the `rospy` package, which is the Python client library for ROS.

- `from std_msgs.msg import String`: Importing the `String` message type from the `std_msgs` package.

- def talker():: Defines a function named `talker()`.

- Inside the `talker()` function:
  - `rospy.Publisher('chatter', String, queue_size=10)`: Creates a publisher object named pub that publishes messages of type String to the topic named 'chatter'.

  - `rospy.init_node('talker', anonymous=True)`: Initializes a ROS node named `'talker'`. The `anonymous=True` argument ensures that if another node with the same name is launched, the node name will be modified to make it unique.

  - `rate = rospy.Rate(10)`: Creates a Rate object that controls the loop rate of the while loop to 10 times per second (10 Hz).

  - The `while` loop runs until the ROS node is shutdown. Inside the loop:
    - `hello_str` = `"hello world"`: Creates a string message **"hello world"**.
    - `rospy.loginfo(hello_str)`: Logs the message to the ROS console at INFO level.
    - `pub.publish(hello_str)`: Publishes the message to the topic 'chatter'.
    - `rate.sleep()`: Sleeps to maintain the specified loop rate.
- `if __name__=='__main__':`: Checks if the script is being run directly. If so, it calls the `talker()` function. It catches the `ROSlnterruptException` exception, which is raised when the node is interrupted or shutdown, and passes without doing anything. This ensures that the script exits gracefully without throwing an error when it's terminated. 

---

[Subscriber](../ros_topic_and_messages_pkg/script/subscribe.py)
```py
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
```
- `#!/usr/bin/env python3`: This line is a shebang that tells the operating system to run this script using the Python 3 interpreter.

- `import rospy`: This imports the rospy library, which is the Python client library for ROS.

- `from std_msgs.msg import String`: This line imports the String message type from the std_msgs package. String messages are simple messages that contain a single string.

- `def listener():`: This function listener() is defined to initialize the ROS node and subscribe to the 'chatter' topic.

  - `rospy.init_node('listener', anonymous=True)`: This line initializes a ROS node name `'listener'`. The `anonymous=True` argument ensures that each instance of the node has a unique name.

  - `rospy.Subscriber('chatter', String, callback)`: This line creates a subscriber that listens to messages on the 'chatter' topic. It specifies that the messages will be of type String and that the callback function to be called when a message is received is callback.

  - `rospy.spin()`: This function keeps the node running until it is explicitly stopped. It prevents the program from exiting immediately.

- `def callback(data):`: This is the callback function that gets called whenever a new message is received on the 'chatter' topic.

  - `rospy.loginfo('I heard %s' % data.data)`: This line logs the received message to the ROS logging system. It prints the received message to the console with the prefix 'I heard'.

- `if __name__=='__main__':`: This line checks if the script is being run as the main program. If it is, it calls the listener() function to start the ROS node.

![alt text](../images/6.png)

## [<-Back to main](../ros_topic_and_messages.md)

