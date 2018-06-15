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

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  JobStatus.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  PickPlace.srv
  MoveLin.srv
  MovePTP.srv
  MoveJoints.srv
  JobEnv.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)
