# 2.ROS_Basics_Workshop

### contain all data related to the Workshop topic.

## [A - ROS introduction and installation](ROS_introduction_and_installation/ROS_intro.md)

The Robot Operating System (**ROS**) is an open-source, meta-operating system designed to provide a structured framework for robotics software development. It offers a collection of tools and libraries to help in creating complex and robust robot behavior across a wide variety of robotic platforms. ROS facilitates hardware abstraction, low-level device control, and implementation of commonly-used functionality, which streamlines the process of building and programming robots. It's widely used in research and industry, providing a rich set of capabilities for robot developers


## [B - ROS Messages and Topics](ROS-Messages-and-Topics/ros_topic_and_messages.md)


**`ROS Node`**: A computational unit in ROS that can communicate by publishing or subscribing to topics, providing or using services, and accessing the parameter server.

**`ROS Topic`**: A named channel for communication between ROS nodes, allowing asynchronous message passing. Nodes can publish messages to topics or subscribe to receive messages from topics.

**`ROS Message (Msg)`**: A data structure used to represent information exchanged between ROS nodes on topics. Messages are defined in .msg files and specify the format and content of data being transmitted.
## [C - ROS Service and Action](ROS-Services-and-Actions/ros_services_and_action.md)

**`ROS Service`**: AREAS service allows nodes to send a request and receive a response. It follows asynchronous communication pattern, meaning the requester waits for a response from the service provider before proceeding. Services are useful for tasks that require immediate feedback or where the requester needs specific information or actions to be performed.

**`ROS Action`**: OR actions are similar to services but follow an asynchronous communication pattern. With actions, a node can send a goal to another node and continue its own processing without waiting for a response. The node providing the action periodically sends updates on the progress of the goal and, eventually, a result when the goal is completed. Actions are commonly used for tasks that involve lengthy computations, motion planning, or tasks with multiple steps.


