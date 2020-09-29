cmake_minimum_required(VERSION 2.8.3)
project(rll_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib actionlib_msgs
  geometry_msgs
  message_generation
  rll_core
  std_msgs
  std_srvs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  JobExtraField.msg
  JobStatus.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  GetJointValues.srv
  GetPose.srv
  MoveGripper.srv
  MoveJoints.srv
  MoveLin.srv
  MoveLinArmangle.srv
  MovePTP.srv
  MovePTPArmangle.srv
  MoveRandom.srv
  PickPlace.srv
  PickPlaceHere.srv
  ValidatePickPlace.srv
)

add_action_files(DIRECTORY action FILES
  JobEnv.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_runtime rll_core std_msgs std_srvs
)
