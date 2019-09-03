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
from .util import apply_ansi_format, C_OK, C_FAIL, C_WARN, C_INFO

MOVE_ERRORS = {
    0: "SUCCESS",

    # user/input error
    1: "INVALID_INPUT",
    2: "JOINT_VALUES_OUT_OF_RANGE",
    3: "OUTSIDE_WORKSPACE",
    4: "INVALID_TARGET_POSE",
    5: "IMPOSSIBLE_MOTION",
    6: "TOO_FEW_WAYPOINTS",
    7: "GOAL_TOO_CLOSE_TO_START",
    8: "GOAL_IN_COLLISION",

    # non critical failure
    64: "RECOVERABLE_FAILURE",
    65: "MOVEIT_PLANNING_FAILED",
    66: "ONLY_PARTIAL_PATH_PLANNED",
    67: "TRAJECTORY_MODIFICATION_FAILED",
    68: "NO_RANDOM_POSITION_FOUND",

    # critical failure
    128: "CRITICAL_FAILURE",
    129: "MOVEMENT_NOT_ALLOWED",
    130: "EXECUTION_FAILED",
    131: "MANIPULATOR_NOT_AVAILABLE",
    132: "GRIPPER_OPERATION_FAILED",
    133: "GRIPPER_DID_NOT_ACKNOWLEDGE",
    134: "GRIPPER_MOVEMENT_FAILED",
    135: "RESET_TO_HOME_FAILED",

    255: "NOT_SET"
}


class ErrorCode(object):
    OK = 0
    INVALID_INPUT = 1
    RECOVERABLE_FAILURE = 64
    CRITICAL = 128

    def __init__(self, code):
        self.code = code

    def is_invalid_input(self):
        return ErrorCode.INVALID_INPUT <= self.code < ErrorCode.RECOVERABLE_FAILURE

    def is_recoverable_failure(self):
        return ErrorCode.RECOVERABLE_FAILURE <= self.code < ErrorCode.CRITICAL

    def is_critical_failure(self):
        return self.code >= ErrorCode.CRITICAL

    def __nonzero__(self):
        raise UserWarning("Don't use the implicit conversion to bool. Use the "
                          "provided methods, e.g. succeeded() instead.")

    def succeeded(self):
        return self.code == ErrorCode.OK

    def failed(self):
        return self.code == ErrorCode.OK

    def __repr__(self):
        if self.code in MOVE_ERRORS:
            color = C_OK
            if self.is_critical_failure():
                color = C_FAIL
            elif self.is_recoverable_failure():
                color = C_WARN
            elif self.is_invalid_input():
                color = C_INFO

            return apply_ansi_format(MOVE_ERRORS[self.code], color)

        return "Unknown error code"


class ServiceCallFailure(Exception):

    def __init__(self, error_code, *args):
        self.error_code = error_code
        Exception.__init__(self, *args)

    def __repr__(self):
        return "%s: %s" % (self.__class__.__name__, Exception.__repr__(self))

    def __str__(self):
        return "%s: %s" % (self.__class__.__name__,  Exception.__str__(self))


class CriticalServiceCallFailure(ServiceCallFailure):

    def __init__(self, error_code, *args):
        ServiceCallFailure.__init__(self, error_code, *args)


if __name__ == "__main__":
    e_ok = ErrorCode(0)
    e_user_fail = ErrorCode(1)
    e_non_critical = ErrorCode(64)
    e_critical = ErrorCode(128)

    def code_check(error_code):
        if error_code:
            print ("if e: => True")
        else:
            print ("if e: => False")

        print("code: %s, succ: %s, crit: %s, non-crit: %s, inv: %s" % (
            error_code, error_code.succeeded(),
            error_code.is_critical_failure(),
            error_code.is_recoverable_failure(),
            error_code.is_invalid_input()))

    # basic checks to validate the ErrorCode class
    code_check(e_ok)
    code_check(e_user_fail)
    code_check(e_non_critical)
    code_check(e_critical)
