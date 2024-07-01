## Creating custom services
### 1- Define a custom services 
Define a service **`message`**: You need to define a service message that specifies the request and response types. Create a file named [**MyService.srv**](../ros_server_pkg/srv/MyService.srv) in the [**srv**](../ros_server_pkg/srv) directory of your package ,and define your custom service message.


```bash
# MyService.srv
string input
---
string output

```
### 2- Compile the services
Make sure your **`CMakeLists.txt`** contains the following lines to ensure that your custom message is compiled:

```Cpp
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_service_files(
  FILES
  MyService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
```

### 3-Write server nodes
[**server.py**](../ros_server_pkg/script/server.py)

```py
#!/usr/bin/env python3

import rospy
from name_of_pkg.srv import MyService

def handle_request(request):
    rospy.loginfo("Received request: %s", request.input)
    response = "Received: " + request.input
    return response

def server():
    rospy.init_node('my_server')
    rospy.Service('my_service', MyService, handle_request)
    rospy.loginfo("Server is ready to receive requests.")
    rospy.spin()

if __name__ == '__main__':
    server()
```
