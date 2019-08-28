/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2018 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
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

#include <rll_move/move_iface_simulation.h>

RLLErrorCode RLLSimulationMoveIface::closeGripper()
{
  if (!manipCurrentStateAvailable())
  {
    return RLLErrorCode::MANIPULATOR_NOT_AVAILABLE;
  }
  ROS_INFO("Closing the gripper");

  gripper_move_group_.setStartStateToCurrentState();
  gripper_move_group_.setNamedTarget(GRIPPER_CLOSE_TARGET_NAME);

  RLLErrorCode error_code = runPTPTrajectory(gripper_move_group_, true);
  if (error_code.failed())
  {
    ROS_INFO("Failed to close the gripper");
  }

  return error_code;
}

RLLErrorCode RLLSimulationMoveIface::openGripper()
{
  if (!manipCurrentStateAvailable())
  {
    return RLLErrorCode::MANIPULATOR_NOT_AVAILABLE;
  }
  ROS_INFO("Opening the gripper");

  gripper_move_group_.setStartStateToCurrentState();
  gripper_move_group_.setNamedTarget(GRIPPER_OPEN_TARGET_NAME);

  RLLErrorCode error_code = runPTPTrajectory(gripper_move_group_, true);
  if (error_code.failed())
  {
    ROS_INFO("Failed to open the gripper");
  }

  return error_code;
}

bool RLLSimulationMoveIface::modifyLinTrajectory(moveit_msgs::RobotTrajectory& /*trajectory*/)
{
  return true;
}

bool RLLSimulationMoveIface::modifyPtpTrajectory(moveit_msgs::RobotTrajectory& /*trajectory*/)
{
  return true;
}
