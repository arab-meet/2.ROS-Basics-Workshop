cmake_minimum_required(VERSION 3.0.2)
project(service_action_examples_package)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  rospy
  std_msgs
)



## Declare a C++ executable for the server
add_executable(square_server src/square_server.cpp)
add_dependencies(square_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(square_server
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for the client
add_executable(square_client src/square_client.cpp)
add_dependencies(square_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(square_client
  ${catkin_LIBRARIES}
)