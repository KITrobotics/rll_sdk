cmake_minimum_required(VERSION 2.8.3)
project(rll_tools)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

catkin_install_python(PROGRAMS scripts/project_runner.py scripts/submit_project.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})