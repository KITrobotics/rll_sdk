cmake_minimum_required(VERSION 2.8.3)
project(rll_move)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rll_msgs
  actionlib
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES rll_move
   CATKIN_DEPENDS message_runtime rll_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/move_iface.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(move_iface_full src/move_iface_full_node.cpp)
target_link_libraries(move_iface_full rll_move ${catkin_LIBRARIES})
