cmake_minimum_required(VERSION 2.8.12)
project(rll_moveit_kinematics_plugin)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  rll_kinematics
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rll_kinematics
)

add_library(${PROJECT_NAME}
  src/moveit_kinematics_plugin.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  FILES
  moveit_plugin.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
