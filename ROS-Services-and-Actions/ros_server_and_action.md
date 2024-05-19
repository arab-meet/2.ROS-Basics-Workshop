
# ROS Services



A ROS **`service`** is a way for nodes in a ROS system to **`request`** and **`receive`** information
from other nodes. 

Services are defined using the ROS service definition language and
are similar to messages in that they are simple data structures, but they include a
request and a response message.

A service request is sent by a client node to a server node, and the server node
responds with a service response. The request and response messages have specific
fields defined in the service definition file.

- Communication on service is bidirectional synchronous
communication between the service client requesting a
- service and the service server responding to the request.
Unlike topics, services have no publishers or subscribers.

<p align="center">
<img src="images/15.gif">

ROS has two command-line tools to work with services, **`rossrv`** and **`rosservice`** .

With rossrv , we can see information about the services data structure, and it has
the exact same usage as rosmsg .
With rosservice , we can list and query services. The commands supported are
as follows: 
- **`rosservice args /service`** : This prints service arguments
- **`rosservice call /service args`** : This calls the service with the
provided arguments
- **`rosservice find msg-type`** : This finds services by the service type
- **`rosservice info /service`** : This prints information about the service
- **`rosservice list`** : This lists the active services
- **`rosservice type /service`** : This prints the service type
- **`rosservice uri /service`** : This prints the service ROSRPC URI
---

We are going to list the services available for the **`turtlesim`** node by using the
following code, so if it is not working, run **`roscore`** and **`run`** the **`turtlesim`** node:

```bash
rosservice list
```
You will obtain the following output:

```bash
/clear
/kill
/reset
/rosout/get_loggers
/rosout/set_logger_level
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/get_loggers
/turtlesim/set_logger_level
```
If you want to see the type of any service, for example, the /clear service, use:
```bash
rosservice type /clear
```
You will then obtain:
```bash
std_srvs/Empty
```
To invoke a service, you will use **rosservice call [service] [args]** . If you want
to invoke the `/clear` service, use:
```bash
rosservice call /clear
```
In the `turtlesim` window, you will now see that the `lines` created by the movements
of the turtle will be `deleted`.

<p align="center">
<img src="images/7.png">

Now, we are going to try another service, for example, the `/spawn` service. This
service will `create` another turtle in another location with a different orientation.

To start with, we are going to see the following type of message:
```bash
rosservice type /spawn | rossrv show
```
We will then obtain the following:

```bash
float32 x
float32 y
float32 theta
string name
---
string name
```
With these fields, we know how to invoke the service. We need the positions of
x and y, the orientation (theta), and the name of the new turtle:
```bash
rosservice call /spawn 1.5 2.5 0.0 "new_turtle_2"
```
We then obtain the following result:

<p align="center">
<img src="images/8.png">

## Using the Parameter Server
The Parameter Server is used to store data that is accessible by all the nodes. 

ROS has a tool to manage the Parameter Server called rosparam . The accepted parameters are as follows:
- **rosparam set parameter** value : This sets the parameter
- **rosparam get parameter** : This gets the parameter
- **rosparam load file** : This loads parameters from the file
- **rosparam dump file** : This dumps parameters to the file
- **rosparam delete parameter** : This deletes the parameter
- **rosparam list** : This lists the parameter names

For example, we can see the parameters in the server that are used by all the nodes:

```bash
rosparam list
```
We obtain the following output:
```bash
/rosdistro
/roslaunch/uris/host_ubuntu__37307
/rosversion
/run_id
/turtlesim/background_b
/turtlesim/background_g
/turtlesim/background_r
```
The `background` parameters are of the `turtlesim` node. 
These parameters change the color of the windows that are initially blue. If you want to read a value, you will
use the get parameter:
```bash
rosparam get /turtlesim/background_r
```
To set a new value, you will use the set parameter:
```bash
rosparam set /turtlesim/background_r 150 
```
We then obtain the following result:
<p align="center">
<img src="images/9.png">

### [Exmaple To Creating custom services](source/exmaple_custom_services.md ) 
---
### [Service Task](source/task_custom_service/Task_service.md)
---

### Action Server:
An action server is a node that implements an action.

It receives goal requests from clients, executes the requested task, and sends feedback and results back to the client.

An action server can handle multiple requests concurrently, allowing it to perform multiple tasks simultaneously

### Action Client: 
An action client is a node that sends requests to an action server to execute a specific action. 

It sends a goal to the action server, monitors the feedback provided by the server during the execution of the action, and receives the result once the action is complete. 

The action client allows the user to interact with the action server asynchronously, meaning the client can continue its operations while waiting for the action to finish.

<p align="center">
<img src="images/16.gif"> 


### [Exmaple To Creating action server/clients](/source/example_action.md )
---

### [Action Task](source/task_custom_action/Task_action.md)

