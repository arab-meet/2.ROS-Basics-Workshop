# 2.ROS_Basics_Workshop

### contain all data related to the Workshop topic.

## [A - ROS Launch File](launch_file/launch.md)

The launch files are ROS tools that allows the creation of a list with all the functionalities that
we want to start together with a single command.
As we already noticed at this point of the course, several commands must be executed in several terminals.
For example, one need to start the ROS master.
So with the command the ROS core, then it might be also useful to load some parameters into the parameter
server and also several other nodes can be started .

## [B - ROS Bags](rosbag/bag_file.md)


**`ROS Node`**: A computational unit in ROS that can communicate by publishing or subscribing to topics, providing or using services, and accessing the parameter server.

**`ROS Topic`**: A named channel for communication between ROS nodes, allowing asynchronous message passing. Nodes can publish messages to topics or subscribe to receive messages from topics.

**`ROS Message (Msg)`**: A data structure used to represent information exchanged between ROS nodes on topics. Messages are defined in .msg files and specify the format and content of data being transmitted.
