/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2020 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <rll_kinematics/redundancy_resolution.h>

int main()
{
  // a solver and options instance is always needed
  RLLRedundancyResolution solver;
  RLLInvKinOptions ik_options;

  // set limb lengths, joint position/velocity/acceleration limits
  RLLKinLimbs limb_lengths = { 0.34, 0.4, 0.4, 0.126 };
  RLLKinJointLimits joint_position_limits;
  RLLKinJoints joint_velocity_limits, joint_acceleration_limits;
  joint_position_limits.lower = { -2.93215, -2.05949, -2.93215, -2.05949, -2.93215, -2.05949, -3.01942 };
  joint_position_limits.upper = { 2.93215, 2.05949, 2.93215, 2.05949, 2.93215, 2.05949, 3.01942 };
  joint_velocity_limits = { 1.7104, 1.7104, 1.7453, 2.2689, 2.4434, 3.1415, 3.1415 };
  joint_acceleration_limits = { 5.4444, 5.4444, 5.5555, 7.2222, 7.7777, 10.0, 10.0 };

  RLLKinMsg result =
      solver.initialize(limb_lengths, joint_position_limits, joint_velocity_limits, joint_acceleration_limits);
  // error() contains all possible error states, see the types_utils.h header for the different error and warning states
  if (result.error())
  {
    std::cout << "error: failed to initialize the kinematics solver" << std::endl;
  }

  // multiple solutions can returned, first entry in the solutions vector is the solution closest to the seed state
  RLLKinSolutions solutions;
  // struct containing goal pose, arm angle and global configuration
  RLLKinPoseConfig eef_pose;
  // set goal position
  eef_pose.pose.setPosition(0.5, -0.2, 0.2);
  // set goal orientation using Euler angles (quaternions are also supported, use setQuaternion(double w, double x,
  // double y, double z) for those)
  eef_pose.pose.setRPY(0.0, M_PI, M_PI / 2);
  // seed state contains joint angles from previous time steps
  RLLKinSeedState seed_state;
  // seed state from two previous time steps are required for the redundancy resolution
  seed_state.push_back({ 0.0, 0.03, 0.0, -M_PI / 2, 0.0, M_PI / 2, 0.0 });
  seed_state.push_back({ 0.0, 0.03, 0.0, -M_PI / 2, 0.0, M_PI / 2, 0.0 });

  // set options
  ik_options.method =
      RLLInvKinOptions::RESOLUTION_MULTI_OBJECTIVE;  // default method using multi-objective optimization
  ik_options.joint_velocity_scaling_factor = 0.4;
  ik_options.joint_acceleration_scaling_factor = 0.4;
  ik_options.delta_t_desired = 0.04;  // tuning parameter for redundancy resolution

  // call inverse kinematics
  result = solver.ik(seed_state, &eef_pose, &solutions, ik_options);
  std::cout << "solution with redundancy resolution based on multi-objective optimization" << std::endl;
  std::cout << "result " << result.message() << ", values: " << solutions.front() << std::endl << std::endl;

  // request solution for a specific arm angle
  ik_options.method = RLLInvKinOptions::ARM_ANGLE_FIXED;
  // set desired arm angle
  eef_pose.arm_angle = M_PI / 4;

  result = solver.ik(seed_state, &eef_pose, &solutions, ik_options);
  std::cout << "solution with fixed arm angle" << std::endl;
  std::cout << "result " << result.message() << ", values: " << solutions.front() << std::endl << std::endl;

  // By default, the closest global configs are searched. Here we want to stay in the current global configuration.
  ik_options.global_configuration_mode = RLLInvKinOptions::KEEP_CURRENT_GLOBAL_CONFIG;
  result = solver.ik(seed_state, &eef_pose, &solutions, ik_options);
  std::cout << "solution with fixed global configuration" << std::endl;
  std::cout << "result " << result.message() << ", values: " << solutions.front() << std::endl << std::endl;

  // call forward kinematics
  result = solver.fk(solutions.front(), &eef_pose);
  std::cout << "forward kinematics with solution from last inverse kinematics call" << std::endl;
  std::cout << "result " << result.message() << ", position:" << std::endl;
  std::cout << eef_pose.pose.pos() << std::endl;

  exit(EXIT_SUCCESS);
}
