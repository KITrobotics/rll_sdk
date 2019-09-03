cmake_minimum_required(VERSION 2.8.3)
project(rll_move_client)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS  
  roscpp
  rospy
  moveit_core
  std_msgs
  actionlib
  rll_msgs
  rll_move
)

catkin_python_setup()

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES rll_move_client   
   CATKIN_DEPENDS rll_msgs rll_move
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/move_client.cpp
  src/move_client_util.cpp
  src/move_client_action.cpp  
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(move_client_example src/move_client_example.cpp)
target_link_libraries(move_client_example ${PROJECT_NAME} ${catkin_LIBRARIES})
