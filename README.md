# 2.ROS_Basics_Workshop

### contain all data related to the Workshop topic.

## [A - ROS Launch File](launch_file/launch.md)

The launch files are ROS tools that allows the creation of a list with all the functionalities that
we want to start together with a single command.
As we already noticed at this point of the course, several commands must be executed in several terminals.
For example, one need to start the ROS master.
So with the command the ROS core, then it might be also useful to load some parameters into the parameter
server and also several other nodes can be started .

## [B - ROS Messages and Topics](ROS-Messages-and-Topics/ros_topic_and_messages.md)


**`ROS Node`**: A computational unit in ROS that can communicate by publishing or subscribing to topics, providing or using services, and accessing the parameter server.

**`ROS Topic`**: A named channel for communication between ROS nodes, allowing asynchronous message passing. Nodes can publish messages to topics or subscribe to receive messages from topics.

**`ROS Message (Msg)`**: A data structure used to represent information exchanged between ROS nodes on topics. Messages are defined in .msg files and specify the format and content of data being transmitted.
## [C - ROS Service and Action](ROS-Services-and-Actions/ros_server_and_action.md)

**`ROS Service`**: AREAS service allows nodes to send a request and receive a response. It follows asynchronous communication pattern, meaning the requester waits for a response from the service provider before proceeding. Services are useful for tasks that require immediate feedback or where the requester needs specific information or actions to be performed.

**`ROS Action`**: OR actions are similar to services but follow an asynchronous communication pattern. With actions, a node can send a goal to another node and continue its own processing without waiting for a response. The node providing the action periodically sends updates on the progress of the goal and, eventually, a result when the goal is completed. Actions are commonly used for tasks that involve lengthy computations, motion planning, or tasks with multiple steps.


