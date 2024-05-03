# Table of Contents

- [Ros Nodes](#ros-nodes)
- [Ros Topics](#ros-topics)
- [Ros Messages](#ros-messages)

# Ros Nodes


<p align="center">
<img src="images/node.gif">

**`Nodes`** are executables that can communicate with other processes using topics,
services, or the Parameter Server.

Using nodes in ROS provides us with fault tolerance and separates the code and functionalities making the system simpler.

A node **`must`** have a **`unique`** name in the system. This name is used to permit the
node to communicate with another node using its name without ambiguity.

A node can be written using different libraries such as roscpp and rospy; roscpp is for C++ and rospy is for Python. Throughout this book, we will be using roscpp.

ROS has tools to handle nodes and give us information about it such as rosnode.
The tool rosnode is a command-line tool for displaying information about nodes,
such as listing the currently running nodes. The commands supported are as follows:

- **`rosnode info /node`** : This prints information about the node
- **`rosnode list`** : This lists the active nodes
- **`rosnode kill /node`** : This kills a running node or sends a given signal
- **`rosnode machine hostname`**: This lists the nodes running on a particular
  machine or lists the machines
- **`rosnode ping /node`**: This tests the connectivity to the node
- **`rosnode cleanup`**: This purges registration information from
  unreachable nodes

---

### we are going to use a typical package called `turtlesim`.

### Before starting with anything, you must start `roscore` as follows:

```bash
roscore
```

### Now we are going to start a new node with `rosrun` as follows:

```bash
rosrun turtlesim turtlesim_node
```

### We will then see a new window appear with a little turtle in the middle, as shown

in the following screenshot:

<p align="center">
<img src="images/1.png">

we will see the active node now, used this command:

```bash
rosnode list 
```

The preceding command line prints the following information:

```
/rosout
/teleop_turtle
/turtlesim
```

You can see a lot of information that can be used to debug your programs, using the
following command:

```bash
rosnode info /turtlesim
```

The preceding command line prints the following information:

```bash
Node [/turtlesim]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /turtle1/color_sensor [turtlesim/Color]
 * /turtle1/pose [turtlesim/Pose]

Subscriptions: 
 * /turtle1/cmd_vel [geometry_msgs/Twist]

Services: 
 * /clear
 * /kill
 * /reset
 * /spawn
 * /turtle1/set_pen
 * /turtle1/teleport_absolute
 * /turtle1/teleport_relative
 * /turtlesim/get_loggers
 * /turtlesim/set_logger_level
```

---

### we are going to control `turtlesim` with teleop_key .

Now run turtle_teleop_key as follows:

```bash
rosrun turtlesim turtle_teleop_key
```

#### Use arrow keys to move the turtle.

- **`Up`** arrow Turtle In Turtle’s **`x`** direction
- **`Down`** arrow Turtle In Turtles’s **`-x`** direction
- **`Right`** arrow Rotate **`CW`**
- **`Left`** arrow Rotate **`CCW`**

With this node, we can move the turtle using the arrow keys, as illustrated in the
following screenshot:

<p align="center">
<img src="images/2.gif">

If you want to see information about the **`/teleop_turtle`**

```bash
rosnode info teleop_turtle
```

The preceding command line prints the following information:

```bash
Node [/teleop_turtle]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /turtle1/cmd_vel [geometry_msgs/Twist]

Subscriptions: None

Services: 
 * /teleop_turtle/get_loggers
 * /teleop_turtle/set_logger_level

```

If you want to see information about the **`turtlesim`**

```bash
rosnode info /turtlesim
```

The preceding command line prints the following information:

```bash
Node [/turtlesim]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /turtle1/color_sensor [turtlesim/Color]
 * /turtle1/pose [turtlesim/Pose]

Subscriptions: 
 * /turtle1/cmd_vel [geometry_msgs/Twist]

Services: 
 * /clear
 * /kill
 * /reset
 * /spawn
 * /turtle1/set_pen
 * /turtle1/teleport_absolute
 * /turtle1/teleport_relative
 * /turtlesim/get_loggers
 * /turtlesim/set_logger_level
```

This means that the first node is publishing a topic that the second node can
subscribe it.

# Ros Topics

Topics are **`pathways`** used by nodes to **`transmit data`**.

Topics can be transmitted without a direct connection between nodes, meaning the production and consumption of data are decoupled.

A topic can have `various` subscribers.

<p align="center">
<img src="images/topic.gif">

Each topic is strongly typed by the ROS message type used to publish it, and nodes
can only receive messages from a matching type. A node can subscribe to a topic
only if it has the same message type.

ROS has a tool to work with topics called rostopic. It is a command-line tool that
gives us information about the topic or publishes data directly on the network.
This tool has the following parameters:

- **`rostopic bw /topic`**: This displays the bandwidth used by the topic.
- **`rostopic echo /topic`**: This prints messages to the screen.
  rostopic find message_type: This finds topics by their type.
- **`rostopic hz /topic`**: This displays the publishing rate of the topic.
- **`rostopic info /topic`**: This prints information about the active topic,
  the topics published, the ones it is subscribed to, and services.
- **`rostopic list`**: This prints information about active topics.
- **`rostopic pub /topic type args`**: This publishes data to the topic.
  It allows us to create and publish data in whatever topic we want,
  directly from the command line.
- **`rostopic type /topic`**: This prints the topic type, that is, the type
  of message it publishes.

## Publishers:

Publishers are nodes that send messages to a specific topic.
Publishers create and send messages at a certain rate or in response to specific events.

They are responsible for generating data or information to be shared with other nodes in the system.

Publishers use the **`rospy.Publisher()`** (for **Python**) or **`ros::Publisher`** (for **C++**) API to advertise the topic they will publish to.

## Subscribers:

Subscribers are nodes that receive messages from a specific topic.

Subscribers process the received messages and perform actions based on the data.
They listen for messages on a particular topic and execute a callback function whenever a new message is received.

Subscribers use the **`rospy.Subscriber()`** (for **Python**) or **`ros::Subscriber`**(for **C++**) API to subscribe to the desired topic and specify the callback function to handle incoming messages.

---

You can see the **`topics`** list  using the following command lines:

```bash
rostopic list
```

The preceding command line prints the following information when run `turtlesim`:

```bash
/rosout
/rosout_agg
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

With the **`echo`** parameter, you can see the information sent by the node.

Run the following command line and use the arrow keys to see what data is
being sent:

```bash
rostopic echo /turtle1/cmd_vel
```

The preceding command line prints the following information:

```bash
linear: 
  x: 0.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: -2.0
```

You can see the type of **`message`** sent by the topic using the following command lines:

```
rostopic type /turtle1/cmd_vel 
```

The preceding command line prints the following information:

```
geometry_msgs/Twist
```

These tools are useful because, with this information, we can publish topics using the
command ,**`rostopic pub [topic] [msg_type] [args]`**:

Moving the Turtle **`Once`**

```bash
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 2.0
  y: 1.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```

<p align="center">
<img src="images/3.gif">

Make the turtle move in a **`circle`**:

```bash
rostopic pub -r 1 /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 2.0
  y: 1.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 2.0" 
```

<p align="center">
<img src="images/4.gif">

## [Exmaple To Creating Publisher and Subscriber](source/exmaple_pub_sub.md)



# Ros Messages

A node sends information to another node using messages that are published
by topics.

The message has a simple structure that uses standard types or types
developed by the user.

Message types use the following standard ROS naming convention: the name of
the package, followed by /, and the name of the .msg file. For example, std_msgs/
msg/String.msg has the message type, std_msgs/String.

ROS has the command-line tool rosmsg to get information about messages.
The accepted parameters are as follows:

- **rosmsg show**: This displays the fields of a message
- **rosmsg list**: This lists all the messages
- **rosmsg package**: This lists all the messages in a package
- **rosmsg packages**: This lists all packages that have the message
- **rosmsg users**: This searches for code files that use the message type
- **rosmsg md5**: This displays the MD5 sum of a message

The message definition can consist of two types: **`fields`** and **`constants`** . The field is
split into field types and field name.

- **`Field`** types is the data type of the transmitting
  message and field name is the name of it.
- **`constants`** define a constant value in the
  message file.

Here is an example of message definitions:

- `int32 number`
- `string name`
- `float32 speed`

## Message types

- **`std_msgs`**: Provides basic message types like String, Int32, Float32, Bool, etc. These are used for simple communication between nodes.
- **`sensor_msgs`**: Contains messages for sensor data such as images (for camera imagess), LaserScan (for laser range data), Imu (for inertial measurement unit data), PointCloud2 (for point cloud data), etc.
- **`geometry_msgs`**: Defines messages for geometric data such as Point, Quaternion, Pose (position and orientation), Twist (linear and angular velocities), Transform (transformations between coordinate frames), etc.
- **`nav_msgs`**: Includes messages related to navigation tasks such as Odometry (robot's position and velocity), Path (sequence of poses representing a path), Map (occupancy grid map), etc.
- **`actionlib_msgs`**: Contains messages for defining actions and monitoring their execution, including GoalID, GoalStatus, etc.
- **`trajectory_msgs`** Defines messages for describing robot trajectories, including JointTrajectory (trajectory for robot joints), MultiDOFJointTrajectory (trajectory for multi-degree-of-freedom joints), etc.

Example:

- `std_msgs/String`
- `std_msgs/Char`
- `sensor_msgs/images`
- `sensor_msgs/LaserScan`
- `geometry_msgs/Pose`
- `nav_msgs/Odometry`
- `actionlib_msgs/GoalStatus`
- `trajectory_msgs/JointTrajectory`

If you want to see the **`message`** of **`turtlesim`** fields, you can do it with the following command lines:

```bash
rosmsg show turtlesim/Pose 
```

The preceding command line prints the following information:

```bash
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
```


### [Exmaple To Creating custom messages_sensor](source/example_custom_message_sensor.md)


### [Exmaple To Creating custom messages_robot](source/example_custom_message_robot.md)
---

### [Topic Task](source/task_1_pub_sub/topic_task.md)

### [Msg Task](source/task_2_custom_msg/msg_task.md)

---
# [<-Back to main](../README.md)

