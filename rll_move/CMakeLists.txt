cmake_minimum_required(VERSION 2.8.3)
project(rll_move)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rll_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime rll_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(move_iface src/move_iface.cpp)
target_link_libraries(move_iface ${catkin_LIBRARIES})
