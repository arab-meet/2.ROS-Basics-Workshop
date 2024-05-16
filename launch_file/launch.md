# ROS Launch Files 
Author : Mahmoud Okasha
Reviewed By: Mo3taz
## ROS Launch
If you lazy person like me and you don't want run every node every time 
there a better and faster way to run your nodes and other application related to ros like gazebo and rviz and load URDF models, all this with one line 
``here Examble normal way VS launch file Way``


<div style="display: flex; justify-content: center;">
    <div style="text-align: center;">
        <h3>Normal Way</h3>
        <br>
        <img src="./media/normal_way_run_node.gif" alt="First GIF" style="width: 80%;">
    </div>
    <div style="text-align: center;">
        <h3>Launch file way/h3>
        <br>
        <img src="./media/launch_file_way.gif" alt="Second GIF" style="width: 80%;">
    </div>
</div>
### Launch files

The launch files are ROS tools that allows the creation of a list of node  with all the functionalities that
you want start together with a single command.

The launch files are ``XML documents`` that contain a list of tasks, a list of operations that needs to be executed


As a rule, create a launch file in the launch / directory inside the ROS package.
>**note** : you can change pkg_name from light_robot to you pkg name.
```bash
cd ~/catkin_ws/src/light_robot/
mkdir launch
cd launch/
touch pubsub1.launch
```
Launch file start tag ``launch`` and inside this tag write all nodes and params and all things
the area from the start tag `<tag>`
to the close tag `</ tag> `is treated as one element.
```XML

<launch>
<!-- write what you need -->
    .....
</launch>

```

Launch file, for example, can load several parameters into the parameter server and this is done with the tag ``param``.
```XML
<!-- example -->
    <param name="param_1" value="100"/>
```
Also it can execute several ``nodes``, each of them with a tag node and also in our launch file.

In the node tag `<node />`
- **pkg** : writes the ROS package name to which the ROS node belongs.
- **type** : writes the original name of the file of ROS node.
- **name** : is a name you can give as you like. You can rename it so you can run two of the same ROS nodes with 

```XML
<!-- example -->

    <node name="basic_simple_talker" pkg="light_robot" type="publisher.py" output="screen" />
    <node name="basic_simple_listener" pkg="light_robot" type="subscriber.py" output="screen"/>
```
Also it can include other launch files in your launch file

```XML
<!-- example -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">

  </include>
```
also can add arguments ``arg`` to your launch file that can modify the list of nodes or also they can affect whether some nodes are going to be executed or not, depending on the value of the arguments it takes.

these arguments are a way to make the launch file reusable so that the same file can be used to launch different configurations of the same application with eventually different nodes or different parameters based on the values of the arguments of the launch file.

```XML
<!-- example -->
<!-- here if the argument true the gui will executed  -->
<!-- if false the gui not executed -->
  <arg name="gui" default="true" />

```
there are also tag ``group`` 
```XML
<!-- example -->

<arg name="foo" default="true" />

<group if="$(arg foo)">
  <!-- stuff that will only be evaluated if foo is true -->
  <!-- here write nodes or what you want -->
</group>

```
there are other important tag  ``remap`` this tag can remap from topic to other topic

```XML
<!-- example -->
<remap from="/different_topic" to="/needed_topic"/>
```

this all comman tags used in launch files
Here in this launch file if groub_1 is **true** two nodes are published and remap done and also load param_1 in parameter server
if groub_1 is **false** include other file in our launch file and the content in the other launch file will excuted and also load param_1 but with value 10

```XML

<launch>
  <arg name="groub_1" default="true" />

    <group if="$(arg groub_1)">

        <!-- stuff that will only be evaluated if groub_1 is true -->
            <node name="basic_simple_talker" pkg="light_robot" type="publisher.py" output="screen" >
                
                <remap from="/stringTopic" to="/stringTopicRemapped"/>

            </node>
            <node name="basic_simple_listener" pkg="light_robot" type="subscriber.py" output="screen"/>
            <!-- this publisher publish in topic /stringTopic i will remap it to /stringTopicRemapped -->
            <!-- now you will see two topics in rostopic list  /stringTopic , /stringTopicRemapped -->

            
            <!-- load param_1 with value 100 in rosparam server -->
            <param name="param_1" value="100"/>

    </group>
    <group unless="$(arg groub_1)">

        <!-- stuff that will only be evaluated if groub_1 is false -->
    <!-- here include other launch file  -->
    <include file="$(find light_robot)/launch/pubsub1.launch"/>
            
</launch>
```
### Output is
![Alt Text](./media/launch_test_tags.gif)


```bash
rosnode list
```
```
/basic_simple_listener
/basic_simple_talker
/rosout
```
## rqt tools
`rqt` is a powerful framework within the Robot Operating System (ROS) that allows you to create and manage graphical user interface (GUI) tools as plugins.

Commonly Used rqt Plugins:

`rqt_bag`: A tool for inspecting and replaying ROS bag files.

`rqt_console`: Displays log messages from ROS nodes.

`rqt_graph`: Visualizes the ROS computation graph.

`rqt_logger_level`: Adjusts the logging level of ROS nodes.

`rqt_plot`: Plots data from ROS topics for debugging and analysis2.

### rqt_graph

rqt_graph displays the relationships between active nodes and the messages transmitted across the ROS network.

after runnning launch file use this command
```bash
rqt_graph
```
The GUI screen will appear.

![rqt_graph](images1/rqt_graph.png)

### rqt_graph_console
rqt_graph_console is a viewer within the rqt package that displays messages being published to rosout.

after runnning launch file use this command
```bash
rqt_console
```
The GUI screen will appear.

![rqt_console](images1/rqt_console.png)

---


# References:
[ros launch](http://wiki.ros.org/roslaunch)

### [&lt;-Back to main](../README.md)