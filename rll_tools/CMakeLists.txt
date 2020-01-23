cmake_minimum_required(VERSION 2.8.3)
project(rll_tools)

find_package(catkin REQUIRED COMPONENTS
  rll_msgs
)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS rll_msgs)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

catkin_install_python(PROGRAMS scripts/project_runner.py scripts/project_submitter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#catkin_lint: ignore_once missing_file
install(FILES
  config/api-access.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config OPTIONAL)
