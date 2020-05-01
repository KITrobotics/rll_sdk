#! /usr/bin/env python
#
# This file is part of the Robot Learning Lab Move Client
#
# Copyright (C) 2019 Mark Weinreuter <uieai@student.kit.edu>
# Copyright (C) 2020 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
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

from math import pi

from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

from rll_move_client.error import RLLErrorCode
from rll_move_client.util import orientation_from_rpy

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

        goal_pose.position = Point(0, 0, 20)
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

    def test_3_move_lin_sequence(self):
        # similar to Tower of Hanoi movements

        count = 2
        # ensuring same global configuration for IK
        resp = self.client.move_joints(0, pi / 100, 0, -pi / 2, 0, pi / 2, 0)
        self.assert_last_srv_call_success(resp)
        goal_pose = Pose()
        goal_pose.position = Point(.3, -0.2, .2)
        goal_pose.orientation = orientation_from_rpy(0, pi, pi / 2)
        resp = self.client.move_lin(goal_pose)
        self.assert_last_srv_call_success(resp)

        for i in range(count):
            rospy.loginfo("Run %d/%d", i + 1, count)

            goal_pose.position.z = .005
            self.assert_move_lin_success(goal_pose)

            goal_pose.position.z = .1
            self.assert_move_lin_success(goal_pose)

            goal_pose.position.y = .3
            self.assert_move_lin_success(goal_pose)

            goal_pose.position.z = .005
            self.assert_move_lin_success(goal_pose)

            goal_pose.position.z = .3
            self.assert_move_lin_success(goal_pose)

            goal_pose.position.y = -0.3
            self.assert_move_lin_success(goal_pose)

    def test_4_move_lin_sequence_2(self):
        # similar to maze movements from the Path Planning project

        # ensuring same global configuration for IK
        resp = self.client.move_joints(0, pi / 100, 0, -pi / 2, 0, pi / 2, 0)
        self.assert_last_srv_call_success(resp)
        goal_pose = Pose()
        goal_pose.position = Point(-0.4, -0.5, .3)
        goal_pose.orientation = orientation_from_rpy(0, pi, 0)
        self.assert_move_ptp_success(goal_pose)

        goal_pose.position.z = .005
        self.assert_move_lin_success(goal_pose)

        goal_pose.position.x = -0.3
        goal_pose.position.y = -0.5
        goal_pose.orientation = orientation_from_rpy(0, pi, -pi / 2)
        self.assert_move_lin_success(goal_pose)

        goal_pose.position.x = .4
        goal_pose.orientation = orientation_from_rpy(0, pi, 0)
        self.assert_move_lin_success(goal_pose)

        goal_pose.position.x = .3
        goal_pose.position.y = .6
        goal_pose.orientation = orientation_from_rpy(0, pi, pi / 2)
        self.assert_move_lin_success(goal_pose)

        goal_pose.position.x = -0.4
        goal_pose.orientation = orientation_from_rpy(0, pi, 0)
        self.assert_move_lin_success(goal_pose)

        goal_pose.position.x = .4
        goal_pose.position.y = .3
        goal_pose.orientation = orientation_from_rpy(0, pi, -pi / 2)
        self.assert_move_lin_success(goal_pose)

        goal_pose.position.x = .2
        goal_pose.position.y = -0.4
        goal_pose.orientation = orientation_from_rpy(0, pi, 0)
        self.assert_move_lin_success(goal_pose)

        goal_pose.position.z = .4
        self.assert_move_lin_success(goal_pose)

    def test_5_kinematics(self):
        # test cases for the kinematics evaluation

        goal_pose = Pose()

        # E0
        self.client.move_joints(0, pi / 100, 0, -pi / 100, 0, -pi / 100, 0)
        goal_pose.position = Point(.6, -0.5, .4)
        goal_pose.orientation = Quaternion(.0, .7071068, .0, .7071068)
        self.assert_move_ptp_success(goal_pose)
        goal_pose.position.y = .5
        self.assert_move_lin_success(goal_pose)

        # E1
        resp = self.client.move_joints(0.357309, 0.868465, 0.0, -1.27991,
                                       1.77188, 1.27341, -0.607802)
        self.assert_last_srv_call_success(resp)
        goal_pose.position = Point(.4, -0.55, .4)
        goal_pose.orientation = Quaternion(.0, -.71, .71, .001)
        # TODO: does not work sometimes, definitely a bug
        resp = self.client.move_lin(goal_pose)
        if not resp:
            rospy.logerr("kinematics test case E1 failed, fix this!")
        # self.assert_move_lin_success(goal_pose)

        # S0
        # move to exact start pose, this corresponds to
        # Point(.5, -0.17, .7215), Quaternion(.0, .7071068, .0, .7071068)
        resp = self.client.move_joints(1.23871210362, 0.279898227692,
                                       -1.96717460651, -1.74067076534,
                                       1.88960481975, 0.7599100101,
                                       -1.65368055426)
        self.assert_last_srv_call_success(resp)
        goal_pose.position = Point(.3, -0.17, .7215)
        goal_pose.orientation = Quaternion(.0, .7071068, .0, .7071068)
        self.assert_move_lin_success(goal_pose)

        # S1
        self.client.move_joints(0.357309, 0.868465, 0.0, -1.27991, 1.77188,
                                1.27341, -0.607802)
        goal_pose.position = Point(.4, -0.55, .71)
        goal_pose.orientation = Quaternion(.0, -.7, .71, .003)
        self.assert_move_lin_success(goal_pose)

    def test_6_move_ptp_lin_sequence(self):
        # similar to the hello world for the Playground project

        resp = self.client.move_joints(0.0, pi / 4, 0.0, -pi / 4, 0.0,
                                       -pi / 2, 0.0)
        self.assert_last_srv_call_success(resp)

        goal_pose = Pose()
        goal_pose.position = Point(.5, .2, .7)
        goal_pose.orientation.z = 1
        self.assert_move_ptp_success(goal_pose)

        goal_pose.orientation = orientation_from_rpy(0, pi / 2, 0)
        self.assert_move_ptp_success(goal_pose)

        goal_pose.orientation = orientation_from_rpy(pi / 4, pi / 2, 0)
        self.assert_move_ptp_success(goal_pose)

        resp = self.client.move_joints(0.0, pi / 4, 0.0, -pi / 4, 0.0,
                                       -pi / 2, 0.0)
        self.assert_last_srv_call_success(resp)
        goal_pose.position = Point(0.5, -0.6, 0.3)
        goal_pose.orientation = orientation_from_rpy(0, pi / 2, 0)
        self.assert_move_ptp_success(goal_pose)

        goal_pose.position.z = .7
        self.assert_move_lin_success(goal_pose)
        goal_pose.position.y = -0.2
        self.assert_move_lin_success(goal_pose)
        goal_pose.position.y = -0.6
        goal_pose.position.z = .3
        self.assert_move_lin_success(goal_pose)

    def test_7_move_lin_ptp_armangle_config(self):
        self.client.move_joints(0, pi / 100, 0, -pi / 2, 0, pi / 2, 0)
        _, arm_angle, config = self.client.get_current_pose(detailed=True)
        self.assertEqual(config, 2, "wrong config determined")

        goal_pose = Pose()
        goal_pose.position = Point(.6, -0.17, .7215)
        goal_pose.orientation = Quaternion(.0, .7071068, .0, .7071068)
        goal_arm_angle = 0.0
        resp = self.client.move_ptp_armangle(goal_pose, goal_arm_angle)
        self.assert_last_srv_call_success(resp)
        _, arm_angle, config = self.client.get_current_pose(detailed=True)
        self.assertEqual(config, 2, "wrong config for test case")
        self.assert_joint_values_close(arm_angle, goal_arm_angle)

        goal_pose.position = Point(.3, .0, .3)
        goal_pose.orientation = orientation_from_rpy(0, pi, 0)
        self.assert_move_ptp_success(goal_pose)
        goal_arm_angle = pi / 2
        resp = self.client.move_lin_armangle(goal_pose, goal_arm_angle, True)
        self.assert_last_srv_call_success(resp)
        _, arm_angle, config = self.client.get_current_pose(detailed=True)
        self.assert_joint_values_close(arm_angle, goal_arm_angle)
        goal_arm_angle = -pi / 2
        resp = self.client.move_lin_armangle(goal_pose, goal_arm_angle, False)
        self.assert_last_srv_call_success(resp)
        goal_arm_angle = pi / 2
        resp = self.client.move_lin_armangle(goal_pose, goal_arm_angle, True)
        self.assert_last_srv_call_success(resp)

        goal_arm_angle = 0.0
        resp = self.client.move_lin_armangle(goal_pose, goal_arm_angle, False)
        self.assert_last_srv_call_success(resp)
        goal_pose.position.y = .4
        goal_arm_angle = pi / 3
        resp = self.client.move_lin_armangle(goal_pose, goal_arm_angle, True)
        self.assert_last_srv_call_success(resp)
        goal_pose.position.x = .0
        goal_pose.orientation = orientation_from_rpy(0, pi, pi / 2)
        goal_arm_angle = -pi / 2
        resp = self.client.move_lin_armangle(goal_pose, goal_arm_angle, False)
        self.assert_last_srv_call_success(resp)
