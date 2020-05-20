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

#include <rll_move/move_iface_error.h>

const char* stringifyMoveItErrorCodes(const moveit_msgs::MoveItErrorCodes& error_code)
{
  switch (error_code.val)
  {
    case moveit_msgs::MoveItErrorCodes::SUCCESS:
      return "SUCCESS";
    case moveit_msgs::MoveItErrorCodes::FAILURE:
      return "FAILURE";
    case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
      return "PLANNING_FAILED";
    case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
      return "INVALID_MOTION_PLAN";
    case moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case moveit_msgs::MoveItErrorCodes::CONTROL_FAILED:
      return "CONTROL_FAILED";
    case moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:
      return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case moveit_msgs::MoveItErrorCodes::TIMED_OUT:
      return "TIMED_OUT";
    case moveit_msgs::MoveItErrorCodes::PREEMPTED:
      return "PREEMPTED";
    case moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION:
      return "START_STATE_IN_COLLISION";
    case moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
      return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION:
      return "GOAL_IN_COLLISION";
    case moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
      return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
      return "GOAL_CONSTRAINTS_VIOLATED";
    case moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME:
      return "INVALID_GROUP_NAME";
    case moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
      return "INVALID_GOAL_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE:
      return "INVALID_ROBOT_STATE";
    case moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME:
      return "INVALID_LINK_NAME";
    case moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME:
      return "INVALID_OBJECT_NAME";
    case moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE:
      return "FRAME_TRANSFORM_FAILURE";
    case moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE:
      return "COLLISION_CHECKING_UNAVAILABLE";
    case moveit_msgs::MoveItErrorCodes::ROBOT_STATE_STALE:
      return "ROBOT_STATE_STALE";
    case moveit_msgs::MoveItErrorCodes::SENSOR_INFO_STALE:
      return "SENSOR_INFO_STALE";
    case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
      return "NO_IK_SOLUTION";
    default:
      return "UNKNOWN_ERROR";
  };
}

RLLErrorCode convertMoveItErrorCode(const moveit::planning_interface::MoveItErrorCode& error_code)
{
  switch (error_code.val)
  {
    case moveit_msgs::MoveItErrorCodes::SUCCESS:
      return RLLErrorCode::SUCCESS;

    case moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME:
    case moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
    case moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE:
    case moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME:
    case moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME:
      return RLLErrorCode::INVALID_INPUT;

    case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
    case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
      return RLLErrorCode::MOVEIT_PLANNING_FAILED;

    case moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION:
      return RLLErrorCode::GOAL_IN_COLLISION;

    default:
      // treat all not explicitly handled cases as critical
      return RLLErrorCode::CRITICAL_FAILURE;
  }
}

const char* RLLErrorCode::message() const noexcept
{
  switch (value_)
  {
    case SUCCESS:
      return "SUCCESS";
    case NOT_SET:
      return "NOT_SET";
    case INVALID_INPUT:
      return "INVALID_INPUT";
    case JOINT_VALUES_OUT_OF_RANGE:
      return "JOINT_VALUES_OUT_OF_RANGE";
    case OUTSIDE_WORKSPACE:
      return "OUTSIDE_WORKSPACE";
    case INVALID_TARGET_POSE:
      return "INVALID_TARGET_POSE";
    case TOO_FEW_WAYPOINTS:
      return "TOO_FEW_WAYPOINTS";
    case GOAL_TOO_CLOSE_TO_START:
      return "GOAL_TOO_CLOSE_TO_START";
    case GOAL_IN_COLLISION:
      return "GOAL_IN_COLLISION";
    case NO_IK_SOLUTION_FOUND:
      return "NO_IK_SOLUTION_FOUND";
    case RECOVERABLE_FAILURE:
      return "RECOVERABLE_FAILURE";
    case MOVEIT_PLANNING_FAILED:
      return "MOVEIT_PLANNING_FAILED";
    case ONLY_PARTIAL_PATH_PLANNED:
      return "ONLY_PARTIAL_PATH_PLANNED";
    case TRAJECTORY_MODIFICATION_FAILED:
      return "TRAJECTORY_MODIFICATION_FAILED";
    case NO_RANDOM_POSITION_FOUND:
      return "NO_RANDOM_POSITION_FOUND";
    case CONCURRENT_SERVICE_CALL:
      return "CONCURRENT_SERVICE_CALL";
    case INSUFFICIENT_PERMISSION:
      return "INSUFFICIENT_PERMISSION";
    case JOB_EXECUTION_TIMED_OUT:
      return "JOB_EXECUTION_TIMED_OUT";
    case CRITICAL_FAILURE:
      return "CRITICAL_FAILURE";
    case SERVICE_CALL_NOT_ALLOWED:
      return "SERVICE_CALL_NOT_ALLOWED";
    case EXECUTION_FAILED:
      return "EXECUTION_FAILED";
    case MANIPULATOR_NOT_AVAILABLE:
      return "MANIPULATOR_NOT_AVAILABLE";
    case GRIPPER_OPERATION_FAILED:
      return "GRIPPER_OPERATION_FAILED";
    case INTERNAL_ERROR:
      return "INTERNAL_ERROR";

    default:
      return "UNKNOWN_ERROR";
  }
}
