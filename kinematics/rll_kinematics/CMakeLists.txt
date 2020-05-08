cmake_minimum_required(VERSION 2.8.12)
project(rll_kinematics)

find_package(catkin REQUIRED COMPONENTS rll_core)
find_package(Eigen3 REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
include_directories(include)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rll_core
)

add_library(${PROJECT_NAME}
  src/arm_angle_intervals.cpp
  src/forward_kinematics.cpp
  src/inverse_kinematics.cpp
  src/inverse_kinematics_coefficients.cpp
  src/redundancy_resolution.cpp
  src/types_utils.cpp
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_executable(${PROJECT_NAME}_example src/example_usage.cpp)
target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_example RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
