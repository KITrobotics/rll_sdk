cmake_minimum_required(VERSION 2.8.3)
project(rll_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_srvs
  std_msgs
  geometry_msgs
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  PickPlace.srv
  MoveLin.srv
  MoveJoints.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)
