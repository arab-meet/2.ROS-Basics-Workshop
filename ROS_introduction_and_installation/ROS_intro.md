# ROS_introduction_and_installation

Author: Mohamed Elshamy

Review :

# Introduction to ROS

* ROS is an open-source robot operating system.
* A set of software libraries and tools that help you build robot applications that work across a wide variety of robotic platforms
* Originally developed in 2007 at the Stanford Artificial Intelligence
* Laboratory and development continued at Willow Garage
* Since 2013 managed by OSRF (Open Source Robotics Foundation)

### ROS Distributions:
![ros history](<images/ros history.png>)


# ROS architecture and components

ROS (Robot Operating System) is a framework for developing robot software. It consists of three levels of concepts: the Filesystem level, the Computation Graph level, and the Community level1.

## Filesystem Level
The Filesystem level covers the organization of ROS resources on disk, such as:

* **Packages**: Packages are the main unit for organizing software in ROS. A package may contain ROS nodes, libraries, configuration files, or any other resources that are useful for a robot application. A package has a minimum structure and content, such as a package.xml file that provides metadata about the package2.
* **Metapackages**: Metapackages are special packages that only serve to group related packages together. They are useful for defining dependencies and distributing software. For example, the ros_base metapackage includes the core ROS packages3.
* **Message types**: Message types define the data structures for messages that are exchanged between ROS nodes. They are stored in .msg files in the msg directory of a package4.
Service types: Service types define the request and response data structures for services that are offered by ROS nodes. They are stored in .srv files in the srv directory of a package5.
* **Action types**: Action types define the goal, feedback, and result data structures for actions that are performed by ROS nodes. They are stored in .action files in the action directory of a package.
## Computation Graph Level
The Computation Graph level describes the communication between ROS processes that are running on one or more machines. The main concepts are:

![ROS concept](<images/ROS concept.png>)

* **Nodes**: Nodes are executable programs that perform computation and communicate with each other using ROS topics, services, actions, or parameters. Nodes can be written in different languages, such as Python, C++, or Lisp.
* **Master**: The Master is a central process that provides name and registration services for the nodes in the ROS system. It allows nodes to find and communicate with each other. The Master also provides a Parameter Server, which is a shared dictionary of key-value pairs that nodes can access and modify.
* **Topics**: Topics are named buses that nodes can use to exchange messages. A node can publish messages to a topic or subscribe to messages from a topic. Topics are anonymous and asynchronous, meaning that the publisher and subscriber do not need to know each other or wait for each other.

![Nodes & Topics](<images/node and topic.png>)

* **Services**: Services are named pairs of request and response messages that nodes can use to invoke synchronous remote procedure calls. A node can offer a service or call a service. Services are persistent and synchronous, meaning that the service provider and the service client need to be available and wait for each other.
* **Actions**: Actions are named triples of goal, feedback, and result messages that nodes can use to perform long-running tasks. A node can provide an action or execute an action. Actions are persistent and asynchronous, meaning that the action provider and the action client can communicate and monitor the progress of the task without blocking each other.

## Community Level
The Community level encompasses the tools and conventions for sharing and distributing ROS software. The main concepts are:

* **Repositories**: Repositories are collections of packages that share a common version control system, such as Git or SVN. Repositories can be hosted on platforms such as GitHub or Bitbucket, and can be indexed by ROS tools for easy access.
* **Distributions**: Distributions are collections of packages that are tested and released together, such as ROS Kinetic or ROS Melodic. Distributions have a fixed release cycle and support policy, and can be installed from pre-built binaries or source code.
* **[ROS Index](https://index.ros.org/)**: ROS Index is a web portal that provides information and documentation for ROS packages, repositories, and distributions. It also allows users to search, browse, and rate ROS software.
* **[ROS Wiki](https://wiki.ros.org/Documentation)**: ROS Wiki is a collaborative website that hosts tutorials, guides, and specifications for ROS. It also serves as a platform for community feedback and discussion.
* **[Stack exchange](https://robotics.stackexchange.com/)**: Stack exchange is a question-and-answer site that helps users solve problems and learn more about ROS. It also fosters a culture of helping and learning within the ROS community.
## ROS Workspaces and ROS Packages
### ROS Workspaces
ROS Workspace is a concept that helps organize your Robot Operating System (ROS) projects. Think of it as a designated folder where you manage related pieces of ROS code.

The official name for workspaces in ROS is `catkin workspaces`.

#### Components of ROS Workspace
A **Catkin ROS workspace** contains three main spaces:

`src` : contains source code, this will be your main work folder.

`devel` : contains setup files for the project ROS environment.

`build` : contains the compiled binary files.

![ros_WS](images/ros_WS.png)

#### Creating a ROS Workspace
You can create a workspace by open a terminal and execute the following commands:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
### ROS Packages
ROS packages reside in the `src` space. In ROS, software is organised in ROS packages.

A ROS package typically contains the following things:

- `CMakeList.txt`
- `package.xml` 
- `scripts` (This folder contains all Python scripts)
- `src` (This folder contains all C++ source files)
- `msg` (for custom message definitions)
- `srv` (for service message definitions)
- `include` (headers/libraries that are needed as dependencies)
- `config` (configuration files)
- `launch` (provide a more automated way of starting nodes)
- `URDF` (Universal Robot Description Files)
- `meshes` (CAD files in .dae (Collada) or .stl (STereoLithography) format)
- `worlds` (XML like files that are used for Gazebo simulation environments)

![ros_pkgs](images/ros_Pkgs.png)


#### package.xml
The package.xml file defines the properties of the
package:
- Package name
- Version number
- Authors
- Dependencies on other packages

![pkg_xml](images/pkg_xml.png)


#### CMakeLists.txt
The main CMake file to build the package
Calls catkin-specific functions/macros
- "Read" the package.xml
- find other catkin packages to access libraries / include directories
- export items for other packages depending on you

![cmakelist](images/cmakelist.png)


---

#### ROS package example 
There is a turtlesim package

![pkg_example](images/pkg_example.png)


---

#### Creating ROS package
You can create a ROS package by open a terminal and execute the following commands:
```bash
cd ~/catkin_ws/src
catkin_init_workspace
catkin_create_pkg Arab_meetup_robot std_msgs rospy roscpp
```

### Building ROS workspaces and ROS packages

Once your workspace is set up, you can start building packages using the `catkin_make` command.

#### catkin_make
`catkin_make` builds all packages in the workspace.

```bash
cd ~/catkin_ws
catkin_make
```


# ROS installation and setup


To install ROS Neotic run this [script file](https://github.com/arab-meet/2.ROS_Basics_Workshop/blob/pr_intro_into_ros/scripts/ros.sh) or install it manually by the following steps:

* Setup your computer to accept software
from packages.ros.org.
``` bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" >
/etc/apt/sources.list.d/ros-latest.list'
```
``` bash
sudo apt install curl  
```

* Check the system is up-to-date
```bash
sudo apt update
```
```bash
sudo apt install ros-noetic-desktop-full
```
* Then added to your bash

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
```bash
source ~/.bashrc
```
```bash
source /opt/ros/noetic/setup.bash
```
* Install tool and dependencies
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```
* Run some core components in ROS
```bash 
sudo apt install python3-rosdep
```
```bash
sudo rosdep init
```
```bash
rosdep update
```
* To make sure that you have ROS run the following command
```bash
 printenv | grep ROS
```
* And the result should be like this:
```
ROS_ROOT=/opt/ros/noetic/share/ros
ROS_PACKAGE_PATH=/opt/ros/noetic/share
ROS_MASTER_URI=http://localhost:11311
ROS_VERSION=1
ROSLISP_PACKAGE_DIRECTORIES=
ROS_DISTRO=noetic
ROS_ETC_DIR=/opt/ros/noetic/etc/ros
```
# Basic ROS Commands and Tools
ROS (Robot Operating System) is a framework for developing robot software. It provides a set of tools and libraries that simplify creating complex and robust robot behavior across various robotic platforms. In this tutorial, we will learn some of the most common and useful ROS commands and tools that can help us interact with ROS nodes, topics, services, parameters, and more.

## ROS Nodes
Nodes are executable programs that perform computation and communicate with each other using ROS topics, services, actions, or parameters. Nodes can be written in different languages, such as Python, C++, or Lisp. To manage and inspect nodes, we can use the following commands:

`rosnode list`: This command lists the names of all the active nodes in the ROS system.

`rosnode info` <node>: This command prints information about a specific node, such as its publications, subscriptions, services, and connections.

`rosnode ping` <node>: This command tests the connectivity to a node by sending ping requests and measuring the response time.

`rosnode kill` <node>: This command terminates a node by sending a shutdown request.
## ROS Topics
Topics are named buses that nodes can use to exchange messages. A node can publish messages to a topic or subscribe to messages from a topic. Topics are anonymous and asynchronous, meaning that the publisher and subscriber do not need to know each other or wait for each other. To manage and inspect topics, we can use the following commands:

`rostopic list`: This command lists the names of all the active topics in the ROS system.

`rostopic info` <topic>: This command prints information about a specific topic, such as its type, publishers, and subscribers.

`rostopic echo` <topic>: This command prints the messages being sent on a topic to the screen.

`rostopic pub` <topic> <type> <args>: This command publishes a message of a given type and arguments to a topic from the command line.

`rostopic hz` <topic>: This command measures the rate of messages being published on a topic.

## ROS Services
Services are named pairs of request and response messages that nodes can use to invoke synchronous remote procedure calls. A node can offer a service or call a service. Services are persistent and synchronous, meaning that the service provider and the service client need to be available and wait for each other. To manage and inspect services, we can use the following commands:

`rosservice list`: This command lists the names of all the active services in the ROS system.

`rosservice info` <service>: This command prints information about a specific service, such as its type, provider, and callers.

`rosservice call` <service> <args>: This command calls a service with the given arguments and prints the response.

`rosservice type` <service>: This command prints the type of the request and response messages of a service.
## ROS Parameters
Parameters are named values that nodes can use to store and retrieve configuration data. Parameters are stored on a central server called the Parameter Server, which is part of the ROS Master. Parameters can have different types, such as strings, integers, floats, booleans, arrays, or dictionaries. To manage and inspect parameters, we can use the following commands:

`rosparam list`: This command lists the names of all the parameters in the ROS system.

`rosparam get` <param>: This command prints the value of a specific parameter or a namespace of parameters.

`rosparam set` <param> <value>: This command sets the value of a parameter or a namespace of parameters.

`rosparam dump` <file>: This command dumps all the parameters to a YAML file.

`rosparam load` <file>: This command loads all the parameters from a YAML file.
## ROS Tools
In addition to the command-line tools, ROS also provides graphical tools that can help us visualize, debug, and analyze ROS data. Some of the most popular ROS tools are:

`roscore`: The basis nodes and programs for ROS-based systems. A roscore must be running for ROS nodes to communicate.

`rosrun`: Runs a ROS package’s executable with minimal typing.

`rqt`: rqt is a Qt-based framework that integrates various ROS tools into a single interface. It allows us to create customized GUIs using plugins, such as rqt_graph, rqt_console, rqt_plot, rqt_image_view, and more.

`rviz`: rviz is a 3D visualization tool that can display sensor data, robot models, navigation maps, and more. It allows us to interact with the robot and the environment using interactive markers, tools, and panels.

`rosbag`: rosbag is a tool that can record and playback ROS message data. It allows us to store and analyze ROS data offline, such as sensor readings, robot poses, images, and more.

`roslaunch`: roslaunch is a tool that can start multiple nodes at once using launch files. It allows us to configure and manage complex ROS systems, such as setting parameters, remapping topics, and launching other launch files.

# [Next Topic Link]

# References:

### [&lt;-Back to main](../README.md)
