# Robot Learning Lab SDK

Software Development Kit for Robot Learning Lab (RLL) projects

## Components

- Model of a RLL robot work cell
- Gazebo, MoveIt! and ROS Control configuration
- Documentation of the web API
- Standardized runner for all projects
- Move Interface for controlling the robot
  - Ability to send joint or cartesian goal positions
  - Support for pick and place tasks
  - Cartesian or point to point trajectories possible
  - Usable as a base for project-specific interfaces

## Usage

The SDK is intended to be used as a dependency of a RLL project. All components are ROS packages and can be built using Catkin.

The Gazebo simulation can be launched together with MoveIt! and an Rviz visualization using

    roslaunch rll_moveit_config moveit_planning_execution.launch

The Move Interface is started with

    roslaunch rll_move move_iface.launch

Move services are available in the `/iiwa/` namespace, e.g. `/iiwa/pick_place` or `/iiwa/move_lin`.

If you want to improve the web API description, have a look at the [ReadMe](docs/api/README.md).

## ROS Kinetic gotchas

Some of the python packages use the `typing` module which cannot be installed via rosdep on Kinetic. You need to install it manually by running:

  pip install typing

## Acknowledgements

The description, control and Gazebo config was inspired by the [iiwa stack](https://github.com/IFL-CAMP/iiwa_stack).