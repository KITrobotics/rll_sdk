cmake_minimum_required (VERSION 2.8.12)
project (rll_analytical_kinematics)

# required for clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++14)

# Warnings
add_definitions(-Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith -Wredundant-decls)

find_package(catkin REQUIRED COMPONENTS
  moveit_core moveit_ros_planning
  pluginlib
  tf_conversions
)

find_package(Eigen3 REQUIRED)
find_package(LAPACK REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
include_directories(include)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rll_moveit_analytical_kinematics_plugin ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/inverse_kinematics_library.cpp
)

add_library(rll_moveit_analytical_kinematics_plugin 
  src/rll_moveit_analytical_kinematics_plugin.cpp)
  
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

target_link_libraries(rll_moveit_analytical_kinematics_plugin ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

target_compile_options(rll_moveit_analytical_kinematics_plugin PRIVATE -Wno-unused-variable)

install(TARGETS ${PROJECT_NAME} rll_moveit_analytical_kinematics_plugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  FILES
  moveit_plugin.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
