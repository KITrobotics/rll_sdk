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

#ifndef RLL_MOVE_MOVE_IFACE_ERROR_H
#define RLL_MOVE_MOVE_IFACE_ERROR_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

class RLLErrorCode
{
public:
  enum Code : uint8_t
  {
    SUCCESS = 0,
    // invalid input
    INVALID_INPUT_BEGIN = 1,  // note: *_BEGIN constants are used in the range checks below
    INVALID_INPUT = INVALID_INPUT_BEGIN,
    JOINT_VALUES_OUT_OF_RANGE,
    OUTSIDE_WORKSPACE,
    INVALID_TARGET_POSE,
    TOO_FEW_WAYPOINTS,
    GOAL_TOO_CLOSE_TO_START,
    GOAL_IN_COLLISION,
    NO_IK_SOLUTION_FOUND,
    // to support project specific error codes, project designer can return these values
    PROJECT_SPECIFIC_INVALID_1 = 32,
    PROJECT_SPECIFIC_INVALID_2,
    PROJECT_SPECIFIC_INVALID_3,

    // recoverable failure
    RECOVERABLE_FAILURE_BEGIN = 64,
    RECOVERABLE_FAILURE = RECOVERABLE_FAILURE_BEGIN,
    MOVEIT_PLANNING_FAILED,
    ONLY_PARTIAL_PATH_PLANNED,
    TRAJECTORY_MODIFICATION_FAILED,
    NO_RANDOM_POSITION_FOUND,
    CONCURRENT_SERVICE_CALL,
    INSUFFICIENT_PERMISSION,
    JOB_EXECUTION_TIMED_OUT,
    SERVICE_CALL_NOT_ALLOWED,
    INVALID_GRIPPER_OPERATION,
    GRIPPER_CONSTRAINT_FAILED,

    PROJECT_SPECIFIC_RECOVERABLE_1 = 96,
    PROJECT_SPECIFIC_RECOVERABLE_2,
    PROJECT_SPECIFIC_RECOVERABLE_3,

    SERVICE_CALL_CLIENT_ERROR = 127,  // only used client side

    // critical failure
    CRITICAL_FAILURE_BEGIN = 128,
    CRITICAL_FAILURE = CRITICAL_FAILURE_BEGIN,
    EXECUTION_FAILED,
    MANIPULATOR_NOT_AVAILABLE,
    GRIPPER_OPERATION_FAILED,
    GRIPPER_DID_NOT_ACKNOWLEDGE,
    GRIPPER_MOVEMENT_FAILED,
    RESET_TO_HOME_FAILED,
    INTERNAL_ERROR,
    PROJECT_SPECIFIC_CRITICAL_1 = 160,
    PROJECT_SPECIFIC_CRITICAL_2,
    PROJECT_SPECIFIC_CRITICAL_3,

    // initial value, used to determine if a value has been set
    NOT_SET = 255
  };

  RLLErrorCode() noexcept = default;
  RLLErrorCode(bool) = delete;                     // forbid this
  RLLErrorCode(Code code) noexcept : value_(code)  // NOLINT google-explicit-constructor
  {
  }

  RLLErrorCode(int code_value) noexcept : value_(static_cast<Code>(code_value))  // NOLINT google-explicit-constructor
  {
  }

  Code value() const noexcept
  {
    return value_;
  }

  explicit operator Code() const noexcept
  {
    return value_;
  }

  explicit operator uint8_t() const noexcept
  {
    return value_;
  }

  operator bool() = delete;

  bool operator==(const RLLErrorCode& a) const noexcept
  {
    return value_ == a.value_;
  }

  bool operator!=(const RLLErrorCode& a) const noexcept
  {
    return value_ != a.value_;
  }

  RLLErrorCode determineWorse(const RLLErrorCode& other) const
  {
    if (value_ > other.value_)  // a larger value signifies a more critical error
    {
      return *this;
    }
    return other;
  }

  bool succeeded() const noexcept
  {
    return value_ == SUCCESS;
  }

  unsigned char succeededSrv() const noexcept
  {
    return static_cast<unsigned char>(succeeded());
  }

  bool failed() const noexcept
  {
    return value_ != SUCCESS;
  }

  bool isCriticalFailure() const noexcept
  {
    // This includes NOT_SET. NOT_SET is a critical error because it means the error_code has not been set, which should
    // not happen
    return value_ >= CRITICAL_FAILURE_BEGIN;
  }

  bool isNonCriticalFailure() const noexcept
  {
    return !isCriticalFailure();
  }

  bool isRecoverableFailure() const noexcept
  {
    return value_ >= RECOVERABLE_FAILURE_BEGIN && value_ < CRITICAL_FAILURE_BEGIN;
  }

  bool isInvalidInput() const noexcept
  {
    return value_ >= INVALID_INPUT_BEGIN && value_ < RECOVERABLE_FAILURE_BEGIN;
  }

  // NOLINTNEXTLINE readability-function-size
  const char* message() const noexcept;

private:
  Code value_{ NOT_SET };
};

const char* stringifyMoveItErrorCodes(const moveit_msgs::MoveItErrorCodes& error_code);
RLLErrorCode convertMoveItErrorCode(const moveit::planning_interface::MoveItErrorCode& error_code);

#endif  // RLL_MOVE_MOVE_IFACE_ERROR_H
