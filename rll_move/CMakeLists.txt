cmake_minimum_required(VERSION 2.8.3)
project(rll_move)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  moveit_core
  moveit_ros_move_group
  moveit_ros_planning
  moveit_ros_planning_interface
  rll_moveit_kinematics_plugin  #catkin_lint: ignore_once literal_project_name
  rll_msgs
  rostest
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS
   message_runtime
   moveit_ros_planning_interface
   rll_moveit_kinematics_plugin  #catkin_lint: ignore_once literal_project_name
   rll_msgs
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(${PROJECT_NAME}
  src/authentication.cpp
  src/move_iface_base.cpp
  src/move_iface_default.cpp
  src/move_iface_error.cpp
  src/move_iface_planning.cpp
  src/move_iface_services.cpp
  src/move_iface_simulation.cpp
  src/move_iface_state_machine.cpp
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(move_iface_full src/move_iface_full_node.cpp)
target_link_libraries(move_iface_full ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS move_iface_full
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


if(CATKIN_ENABLE_TESTING)

  add_rostest(tests/launch/movement_tests.test)

  add_rostest_gtest(unit_tests_cpp tests/launch/unit_tests_cpp.test tests/src/test_permissions.cpp tests/src/test_state_machine.cpp)
  target_link_libraries(unit_tests_cpp ${PROJECT_NAME} ${catkin_LIBRARIES})

endif()
