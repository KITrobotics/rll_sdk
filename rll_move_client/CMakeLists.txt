cmake_minimum_required(VERSION 2.8.3)
project(rll_move_client)

find_package(catkin REQUIRED COMPONENTS  
  rll_move
  rll_msgs
  rostest
)

catkin_python_setup()

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS rll_move rll_msgs
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/move_client.cpp
  src/move_client_listener.cpp
  src/move_client_util.cpp
)

# install the library for other ROS packages
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

catkin_install_python(PROGRAMS scripts/move_client_example.py scripts/move_client_input.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(move_client_example src/move_client_example.cpp)
target_link_libraries(move_client_example ${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  
  add_rostest(tests/launch/unit_tests_python.test)
  
endif()
