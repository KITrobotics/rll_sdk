cmake_minimum_required (VERSION 2.8.12)
project (rll_kinematics)

# required for clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++14)

# Warnings
add_definitions(-Wall -Wextra -Wpedantic -Wredundant-decls)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
)

find_package(Eigen3 REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
include_directories(include)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rll_moveit_kinematics_plugin ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/arm_angle_intervals.cpp
  src/forward_kinematics.cpp
  src/inverse_kinematics.cpp
  src/inverse_kinematics_coefficients.cpp
  src/redundancy_resolution.cpp
  src/types_utils.cpp
)

add_library(rll_moveit_kinematics_plugin
  src/moveit_kinematics_plugin.cpp
)

target_link_libraries(rll_moveit_kinematics_plugin ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} rll_moveit_kinematics_plugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_executable(${PROJECT_NAME}_example src/example_usage.cpp)
target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_example RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  FILES
  moveit_plugin.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
