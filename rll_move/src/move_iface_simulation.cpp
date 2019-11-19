/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2018-2019 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
 * Copyright (C) 2019 Mark Weinreuter <uieai@student.kit.edu>
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

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <rll_move/move_iface_simulation.h>

bool RLLSimulationMoveIface::modifyPtpTrajectory(moveit_msgs::RobotTrajectory& trajectory)
{
  robot_trajectory::RobotTrajectory rt(manip_model_, manip_move_group_.getName());
  rt.setRobotTrajectoryMsg(*manip_move_group_.getCurrentState(), trajectory);

  trajectory_processing::IterativeParabolicTimeParameterization time_param;
  bool success = time_param.computeTimeStamps(rt, DEFAULT_VELOCITY_SCALING_FACTOR, DEFAULT_ACCELERATION_SCALING_FACTOR);
  if (!success)
  {
    return false;
  }

  rt.getRobotTrajectoryMsg(trajectory);

  return true;
}
