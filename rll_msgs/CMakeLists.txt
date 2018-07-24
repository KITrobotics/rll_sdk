cmake_minimum_required(VERSION 2.8.3)
project(rll_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_srvs
  std_msgs
  geometry_msgs
  genmsg actionlib_msgs actionlib
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  JobStatus.msg
  JobExtraField.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  PickPlace.srv
  MoveLin.srv
  MovePTP.srv
  MoveJoints.srv
)

add_action_files(DIRECTORY action FILES
  DefaultMoveIface.action
  JobEnv.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)
