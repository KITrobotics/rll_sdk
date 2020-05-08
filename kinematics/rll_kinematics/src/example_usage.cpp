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

#include <iterator>

#include <rll_kinematics/redundancy_resolution.h>

int main()
{
  RLLRedundancyResolution solver;
  RLLInvKinOptions ik_options;

  RLLKinLimbs limb_lengths = { 0.34, 0.4, 0.4, 0.126 };
  RLLKinJoints joint_limits_min = { -2.93215, -2.05949, -2.93215, -2.05949, -2.93215, -2.05949, -3.01942 };
  RLLKinJoints joint_limits_max = { 2.93215, 2.05949, 2.93215, 2.05949, 2.93215, 2.05949, 3.01942 };

  solver.initialize(limb_lengths, joint_limits_min, joint_limits_max);

  RLLKinJoints solution;
  RLLKinPoseConfig eef_pose;
  eef_pose.pose.setPosition(0.5, -0.2, 0.2);
  eef_pose.pose.setRPY(0.0, M_PI, M_PI / 2);
  RLLKinJoints seed_state = { 0.0, 0.03, 0.0, -M_PI / 2, 0.0, M_PI / 2, 0.0 };

  RLLKinMsg result = solver.ik(seed_state, &eef_pose, &solution, ik_options);
  std::cout << "solution with redundancy resolution based on exponential function" << std::endl;
  std::cout << "result " << result.message() << ", values: " << solution << std::endl << std::endl;

  ik_options.method = RLLInvKinOptions::ARM_ANGLE_FIXED;
  result = solver.ik(seed_state, &eef_pose, &solution, ik_options);
  std::cout << "solution with fixed arm angle" << std::endl;
  std::cout << "result " << result.message() << ", values: " << solution << std::endl << std::endl;

  ik_options.method = RLLInvKinOptions::POSITION_RESOLUTION_EXP;
  ik_options.keep_global_configuration = true;
  result = solver.ik(seed_state, &eef_pose, &solution, ik_options);
  std::cout << "solution with redundancy resolution based on exponential function and fixed global configuration"
            << std::endl;
  std::cout << "result " << result.message() << ", values: " << solution << std::endl << std::endl;

  result = solver.fk(solution, &eef_pose);
  std::cout << "forward kinematics with solution from last inverse kinematics call" << std::endl;
  std::cout << "result " << result.message() << ", position:" << std::endl;
  std::cout << eef_pose.pose.pos() << std::endl;

  exit(EXIT_SUCCESS);
}
