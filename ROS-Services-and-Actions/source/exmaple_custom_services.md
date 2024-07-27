# Service Example/Practice

In this guide, we'll walk through creating a simple service in ROS that computes the square of a number. We'll provide both Python and C++ examples, demonstrating how to implement the service server and client for this task.

## 1- Creating custom service

### 1.1- Define a custom services

Define a service **`message`**: You need to define a service message that specifies the request and response types. Create a file named
[Square.srv](../service_action_examples_package/srv/Square.srv) in the [**srv**](../service_action_examples_package/srv) directory of your package ,and define your custom service message.

```bash
# Square.srv
string input
---
string output

```

### 1.2- Update CmakeLists.txt

- Make sure your **`CMakeLists.txt`** contains the following lines to ensure that your custom message is compiled:

```Cpp
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_service_files(
  FILES
  Square.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
```

### 1.3- Update Package.xml

- open package.xml and add these two lines :

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>

```

## 2- Creating Server and Client in Python :

### 2.1. Write python server node

[**square_server.py**](../service_action_examples_package/scripts/square_server.py)

```py
#!/usr/bin/env python3

import rospy
from service_action_examples_package.srv import Square, SquareResponse

def handle_square(req):
    # Print the received request number
    rospy.loginfo("Received request: %ld", req.number)
    # Compute the square of the number
    result = req.number * req.number
    # Print the result of the computation
    rospy.loginfo("Returning response: %ld", result)
    # Return the result as a SquareResponse message
    return SquareResponse(result)

def square_server():
    # Initialize the ROS node with the name 'square_server'
    rospy.init_node('square_server')
    # Create a service named 'calculate_square' of type Square
    # This service will use the handle_square function to process requests
    s = rospy.Service('calculate_square', Square, handle_square)
    # Print that the service is ready to handle requests
    rospy.loginfo("Ready to calculate the square of a number.")
    # Keep the node running and processing requests
    rospy.spin()

if __name__ == "__main__":
    # Start the square server
    square_server()
#!/usr/bin/env python3

import rospy
from name_of_pkg.srv import Square

def handle_request(request):
    rospy.loginfo("Received request: %s", request.input)
    response = "Received: " + request.input
    return response

def server():
    rospy.init_node('my_server')
    rospy.Service('my_service', Square, handle_request)
    rospy.loginfo("Server is ready to receive requests.")
    rospy.spin()

if __name__ == '__main__':
    server()
```

### 2.3-Write python client node

[**square_client.py**](../service_action_examples_package/scripts/square_client.py)

```py
#!/usr/bin/env python3

import sys
import rospy
from service_action_examples_package.srv import Square

def square_client(number):
    # Wait for the 'calculate_square' service to become available
    rospy.wait_for_service('calculate_square')
    try:
        # Create a handle to the service
        calculate_square = rospy.ServiceProxy('calculate_square', Square)
        # Call the service with the provided number
        response = calculate_square(number)
        # Return the result from the service response
        return response.square
    except rospy.ServiceException as e:
        # Print error if the service call fails
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    # Check if a number was provided as a command-line argument
    if len(sys.argv) == 2:
        number = int(sys.argv[1])
    else:
        print("Usage: square_client.py [number]")
        sys.exit(1)
  
    # Print the number being requested
    print("Requesting the square of %d" % number)
    # Call the service and print the result
    print("Square: %d" % square_client(number))

```

## 3- Creating Server and Client in Cpp :

### 3.1. Write Cpp server node

[**square_server.**](../service_action_examples_package/scripts/square_server.py)cpp

```cpp
#include <ros/ros.h>
#include <service_action_examples_package/Square.h>

// Callback function for handling service requests
bool handleSquare(service_action_examples_package::Square::Request &req,
                  service_action_examples_package::Square::Response &res)
{
    ROS_INFO("Received request: %d", req.number);
    // Compute the square of the number
    res.square = req.number * req.number;
    ROS_INFO("Returning response: %d", res.square);
    return true;
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "square_server");
    ros::NodeHandle nh;

    // Create a service named 'calculate_square' of type Square
    ros::ServiceServer service = nh.advertiseService("calculate_square", handleSquare);
  
    ROS_INFO("cpp node : Ready to calculate the square of a number.");
  
    // Keep the node running
    ros::spin();

    return 0;
}

```


### 3.1. Write Cpp client node

```cpp
#include <ros/ros.h>
#include <service_action_examples_package/Square.h>

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "square_client");
    ros::NodeHandle nh;
  
    ROS_INFO("spp square_client");

    // Create a service client for 'calculate_square'
    ros::ServiceClient client = nh.serviceClient<service_action_examples_package::Square>("calculate_square");

    if (argc != 2)
    {
        ROS_INFO("Usage: square_client [number]");
        return 1;
    }

    // Create a request object and set the number
    service_action_examples_package::Square srv;
    srv.request.number = atoi(argv[1]);

    // Call the service and check the result
    if (client.call(srv))
    {
        ROS_INFO("The square of %d is %d", srv.request.number, srv.response.square);
    }
    else
    {
        ROS_ERROR("Failed tooooo call service calculate_square");
        return 1;
    }

    return 0;
}


```
