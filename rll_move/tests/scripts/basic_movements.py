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

from geometry_msgs.msg import Pose, Point

from rll_move_client.error import RLLErrorCode
from test_util import TestCaseWithRLLMoveClient


class TestBasicMovements(TestCaseWithRLLMoveClient):

    def test_0_error_codes(self):
        e_ok = RLLErrorCode(RLLErrorCode.SUCCESS)
        e_invalid = RLLErrorCode(RLLErrorCode.INVALID_INPUT)
        e_recoverable = RLLErrorCode(RLLErrorCode.RECOVERABLE_FAILURE)
        e_critical = RLLErrorCode(RLLErrorCode.CRITICAL_FAILURE)

        def check_code(error_code, success, failure, recoverable, invalid):
            self.assertEqual(error_code.succeeded(), success)
            self.assertEqual(error_code.failed(), not success)
            self.assertEqual(error_code.is_critical_failure(), failure)
            self.assertEqual(error_code.is_recoverable_failure(), recoverable)
            self.assertEqual(error_code.is_invalid_input(), invalid)

        # basic checks to validate the ErrorCode class
        check_code(e_ok, True, False, False, False)
        check_code(e_invalid, False, False, False, True)
        check_code(e_recoverable, False, False, True, False)
        check_code(e_critical, False, True, False, False)

    def test_1_pose_outside_workspace(self):
        goal_pose = Pose()
        goal_pose.position = Point(1, 1, 1)
        resp = self.client.move_ptp(goal_pose)
        self.assert_last_srv_call_failed(resp,
                                         RLLErrorCode.NO_IK_SOLUTION_FOUND)

    def test_2_move_joints(self):
        resp = self.client.move_joints(0, 0, 0, 0, 0, 0, 0)
        self.assert_last_srv_call_success(resp)

        resp = self.client.move_joints([1, 0, 0, 0, 0, 0, 0])
        self.assert_last_srv_call_success(resp)

        resp = self.client.move_joints((0, 0, 1, 0, 0, 0, 0))
        self.assert_last_srv_call_success(resp)

        # should be seven values
        resp = self.client.move_joints(0, 0, 1, 0, 0, 0, )
        self.assertFalse(resp)
