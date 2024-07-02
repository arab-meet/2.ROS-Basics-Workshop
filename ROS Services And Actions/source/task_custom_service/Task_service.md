# Task: ROS Service

Before you begin this task, please try to write a similar example in a workshop or on your own. If you succeed in writing it without referring back to the example code, you're ready to start the task.


### Description
Write a ROS service  (`num_1`,`num_2`,`sum`) that the client request `sum` of `num_1` and `num_2` from server (from terminal) and server response the answer to client
### Expected Output
![Expected Output](media/task_3_service.gif)

### Note : 
To use terminal as input to send num_1 and num_2 to server 

- `in client script` :

```py
# your code ...

def add_two_number_client(x, y):
    # your code ...
    
def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_number_client(x, y)))
```

don't see any of tips before you try with yourself and tired from searching  

#### Tips for service
<details>
<summary><b>First Tip</b></summary>

make sure in `CMakeLists` 
1- in find_package : `message_generation`

```bash
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

```
2- in add_service_files: name the file in srv

```bash
add_service_files(
  FILES
  AddTwoNumber.srv
)
```

3- generate_messages: not commmented

```bash
generate_messages(
  DEPENDENCIES
  std_msgs
)
```
make sure in `package.xml` you add

```bash
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>

```
</details>

#### Tips for using service server
<details>
<summary><b>second Tip</b></summary>
1- import srv from your package and don't forget your_serviceResponse

`from your_package.srv import file_name_from_srv_folder your_serviceResponse` 

```py
from custom_service_task_pkg.srv import AddTwoNumber, AddTwoNumberResponse

```

</details>

<details>
<summary><b>Third Tip</b></summary>

1- access data right and make response right
`req.num_1`
`req.num_2`
`AddTwoNumberResponse(result)`

```py
AddTwoNumberResponse(req.num_1 + req.num_2)
```

</details>

[for full package ](custom_service_task_pkg)
