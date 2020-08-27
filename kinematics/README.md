# Robot Learning Lab Kinematics

This folder contains the kinematics library and the Moveit kinematics plugin.

## Kinematics library

The `rll_kinematics` library solves the forward and inverse kinematics for 7-DoF serial manipulators without joint offsets (spherical shoulder and wrist). Robots with such a kinematic structure are for example the KUKA iiwas and LWR IV family of robots.

The implementation is real-time-capable and can be used as part of an online motion planning framework or directly integrated into a controller. The solver is very fast. Depending on the configured options, solutions are found in less than 10 microseconds.

To resolve the redundancy, a closed-form multi-objective optimization approach is used that minimizes joint velocities and accelerations while maximizing the distance to joint limits. The solver can be used for PTP motions and to sample given Cartesian paths. For the path sampling, a small step size (e.g. 1mm) should be used to ensure the optimization is able to produce a smooth arm angle movement. As part of an Cartesian online motion controller, the solver can also be used to determine suitable joint angles for the next time step.

The [example app](rll_kinematics/src/example_usage.cpp) shows the intended usage. Limb lengths and joint position, velocity and acceleration limits are set with `initialize()`. The `fk()` and `ik()` methods are used to solve the forward and inverse kinematics. Joint angles from previous time steps are given as seed state. An `RLLInvKinOptions` object is passed which can be used to set different options.

The most important options are `delta_t_desired` and `global_configuration_mode`. `delta_t_desired` is a tuning parameter for the redundancy resolution. If the parameter is select small, the minimization of joint velocities and accelerations is weighted heavier compared to the joint position limit avoidance. If a large value is used, the focus is put on reachability. As a rule of thumb, set `delta_t_desired` to the maximum time difference between two sampling steps that one expects the robot to take. You can also experiment with different values and some test paths to figure out which value works best for you. We recommend to set the real joint velocity and acceleration limits during initialization and then use the scaling factors `joint_velocity_scaling_factor` `joint_acceleration_scaling_factor` to the set current scaling during runtime. Of course, you can also change `delta_t_desired` during runtime to adapt preferences.

The solver is able to distinguish between global configurations that are defined using the joint angle sign at axis two, four and six. With the option `global_configuration_mode`, the handling of the configurations can be influenced. `KEEP_CURRENT_GLOBAL_CONFIG` stays in the current configuration (useful for path following) and returns one solution. `SELECT_NEAREST_GLOBAL_CONFIG` selects the solution that is closest to the seed state and if the seed state is close to several global configurations, returns several solutions in order of their closeness to the seed state as a vector. `RETURN_ALL_GLOBAL_CONFIGS` tests all global configurations and also returns them in the order of closeness. The last two modes are useful for PTP motions and they cover the complete solution space.

Further options can be found in the [header](rll_kinematics/include/rll_kinematics/redundancy_resolution.h).

The library only depends on Eigen and Boost and has no ROS dependencies. We only build it using catkin as part of the RLL SDK, but it is also usable completely outside the ROS ecosystem.


## Moveit kinematics plugin

If you are using Moveit for motion planning, you can use this plugin as drop-in replacement for the default kinematics solver. The default numerical solver is a lot slower than our solution and it exhibits strong drift effects on the null-space manifold, which leads to the elbow floating to the sides.

After building the plugin, you only need to do a small change to your Moveit config to use this plugin. In the `config/kinematics.yaml` config file, change the `kinematics_solver` setting to `rll_moveit_kinematics/RLLMoveItKinematicsPlugin`. You should have velocity and acceleration limits defined in the `config/joint_limits.yaml` file. Then the robot properties are automatically read from the robot's URDF description and Moveit config.
See our [Moveit config package](../rll_moveit_config/) for reference. 

The plugin integrates with the collision avoidance in Moveit and works for planning PTP movements or Cartesian paths. Using the available wrapper methods, it is also possible to directly call the solver and change options. Check out our [planning interface](../rll_move/src/move_iface_planning.cpp) for an example. Our `move_iface` for the RLL exposes several services for easily requesting PTP or linear movements to goal poses, including the option to specify a desired arm angle. See the [RLL docs](https://rll-doc.ipr.kit.edu/) for more usage information if you want to test it with our lab.
