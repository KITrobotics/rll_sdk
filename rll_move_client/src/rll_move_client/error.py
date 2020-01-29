#! /usr/bin/env python
#
# This file is part of the Robot Learning Lab Move Client
#
# Copyright (C) 2019 Mark Weinreuter <uieai@student.kit.edu>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#


class RLLErrorCode(object):
    SUCCESS = 0

    # user/input error
    INVALID_INPUT = 1
    JOINT_VALUES_OUT_OF_RANGE = 2
    OUTSIDE_WORKSPACE = 3
    INVALID_TARGET_POSE = 4
    TOO_FEW_WAYPOINTS = 5
    GOAL_TOO_CLOSE_TO_START = 6
    GOAL_IN_COLLISION = 7
    NO_IK_SOLUTION_FOUND = 8

    # non critical failure
    RECOVERABLE_FAILURE = 64
    MOVEIT_PLANNING_FAILED = 65
    ONLY_PARTIAL_PATH_PLANNED = 66
    TRAJECTORY_MODIFICATION_FAILED = 67
    NO_RANDOM_POSITION_FOUND = 68
    CONCURRENT_SERVICE_CALL = 69
    INSUFFICIENT_PERMISSION = 70
    JOB_EXECUTION_TIMED_OUT = 71
    SERVICE_CALL_NOT_ALLOWED = 72

    SERVICE_CALL_CLIENT_ERROR = 127  # Note: only used client side

    # critical failure
    CRITICAL_FAILURE = 128
    EXECUTION_FAILED = 129
    MANIPULATOR_NOT_AVAILABLE = 130
    GRIPPER_OPERATION_FAILED = 131
    GRIPPER_DID_NOT_ACKNOWLEDGE = 132
    GRIPPER_MOVEMENT_FAILED = 133
    RESET_TO_HOME_FAILED = 134
    INTERNAL_ERROR = 135

    # not set (initial value)
    NOT_SET = 255

    NAMES = {
        SUCCESS: "SUCCESS",

        # user/input error
        INVALID_INPUT: "INVALID_INPUT",
        JOINT_VALUES_OUT_OF_RANGE: "JOINT_VALUES_OUT_OF_RANGE",
        OUTSIDE_WORKSPACE: "OUTSIDE_WORKSPACE",
        INVALID_TARGET_POSE: "INVALID_TARGET_POSE",
        TOO_FEW_WAYPOINTS: "TOO_FEW_WAYPOINTS",
        GOAL_TOO_CLOSE_TO_START: "GOAL_TOO_CLOSE_TO_START",
        GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
        NO_IK_SOLUTION_FOUND: "NO_IK_SOLUTION_FOUND",

        # non critical failure
        RECOVERABLE_FAILURE: "RECOVERABLE_FAILURE",
        MOVEIT_PLANNING_FAILED: "MOVEIT_PLANNING_FAILED",
        ONLY_PARTIAL_PATH_PLANNED: "ONLY_PARTIAL_PATH_PLANNED",
        TRAJECTORY_MODIFICATION_FAILED: "TRAJECTORY_MODIFICATION_FAILED",
        NO_RANDOM_POSITION_FOUND: "NO_RANDOM_POSITION_FOUND",
        CONCURRENT_SERVICE_CALL: "CONCURRENT_SERVICE_CALL",
        INSUFFICIENT_PERMISSION: "INSUFFICIENT_PERMISSION",
        JOB_EXECUTION_TIMED_OUT: "JOB_EXECUTION_TIMED_OUT",

        # critical failure
        CRITICAL_FAILURE: "CRITICAL_FAILURE",
        SERVICE_CALL_NOT_ALLOWED: "SERVICE_CALL_NOT_ALLOWED",
        EXECUTION_FAILED: "EXECUTION_FAILED",
        MANIPULATOR_NOT_AVAILABLE: "MANIPULATOR_NOT_AVAILABLE",
        GRIPPER_OPERATION_FAILED: "GRIPPER_OPERATION_FAILED",
        GRIPPER_DID_NOT_ACKNOWLEDGE: "GRIPPER_DID_NOT_ACKNOWLEDGE",
        GRIPPER_MOVEMENT_FAILED: "GRIPPER_MOVEMENT_FAILED",
        RESET_TO_HOME_FAILED: "RESET_TO_HOME_FAILED",
        INTERNAL_ERROR: "INTERNAL_ERROR",

        NOT_SET: "NOT_SET"
    }

    HINTS = {
        JOINT_VALUES_OUT_OF_RANGE: ("One or more of the joint values "
                                    "you specified are outside their "
                                    "allowed limits."),
        INVALID_TARGET_POSE: ("The pose/joint values you specified "
                              "cannot be reached i.e. they are outside "
                              "their allowed limits, or would move the "
                              "robot outside the allowed workspace."),
        TOO_FEW_WAYPOINTS: ("The distance to the requested goal pose "
                            "is probably too small (e.g. less than 5mm "
                            "for a linear motion)."),
        GOAL_TOO_CLOSE_TO_START: ("The robot is already at/too close "
                                  "to the goal and no motion is performed."),
        NO_IK_SOLUTION_FOUND: ("The inverse kinematics did not yield a "
                               "solution. Is your goal pose within "
                               "the workspace?"),
        NO_RANDOM_POSITION_FOUND: ("Random pose generation may fail e.g. "
                                   "if the generated pose is in collision."),
        GOAL_IN_COLLISION: ("The request motion would result in a collision "
                            "either with an obstacle or the robot itself."),
        MOVEIT_PLANNING_FAILED: ("This is a generic motion planning error and "
                                 "can be caused e.g. by requesting a pose "
                                 "outside the allowed workspace."),
        ONLY_PARTIAL_PATH_PLANNED: ("A pose between the start and goal pose "
                                    "of a linear motion causes a collision, "
                                    "only part of the motion is possible."),
        SERVICE_CALL_NOT_ALLOWED: ("The movement interface currently does not "
                                   "accept service calls, possibly due to a "
                                   "critical failure.")
    }

    def __init__(self, code=NOT_SET):
        self.code = code

    def is_invalid_input(self):
        return RLLErrorCode.INVALID_INPUT <= self.code < \
            RLLErrorCode.RECOVERABLE_FAILURE

    def is_recoverable_failure(self):
        return RLLErrorCode.RECOVERABLE_FAILURE <= self.code < \
            RLLErrorCode.CRITICAL_FAILURE

    def is_critical_failure(self):
        return self.code >= RLLErrorCode.CRITICAL_FAILURE

    def __nonzero__(self):
        raise UserWarning("Don't use the implicit conversion to bool. Use the "
                          "provided methods, e.g. succeeded() instead.")

    def succeeded(self):
        return self.code == RLLErrorCode.SUCCESS

    def failed(self):
        return self.code != RLLErrorCode.SUCCESS

    def __repr__(self):
        return RLLErrorCode.NAMES.get(self.code,
                                      "UNKNOWN_ERROR_CODE(%s)" % self.code)

    def get_hint(self):
        return RLLErrorCode.HINTS.get(self.code, None)


class ServiceCallFailure(Exception):

    def __init__(self, error_code, *args):
        self.error_code = error_code
        Exception.__init__(self, *args)

    def __repr__(self):
        return "%s: %s" % (self.__class__.__name__, Exception.__repr__(self))

    def __str__(self):
        return "%s: %s" % (self.__class__.__name__, Exception.__str__(self))


class CriticalServiceCallFailure(ServiceCallFailure):

    def __init__(self, error_code, *args):
        ServiceCallFailure.__init__(self, error_code, *args)
