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

from math import pi

from geometry_msgs.msg import Pose, Point
import rospy
import rll_msgs

from rll_move_client.error import RLLErrorCode, CriticalServiceCallFailure
from rll_move_client.tests_util import TestCaseWithRLLMoveClient, \
    concurrent_call
from rll_move_client.util import orientation_from_rpy
from rll_tools.run import idle


class TestInvalidMovements(TestCaseWithRLLMoveClient):

    def test_0_pose_outside_workspace(self):
        goal_pose = Pose()
        goal_pose.position = Point(1, 1, 1)
        resp = self.client.move_ptp(goal_pose)
        self.assert_last_srv_call_failed(
            resp, RLLErrorCode.NO_IK_SOLUTION_FOUND)

    def test_1_move_ptp(self):
        goal_pose = Pose()
        goal_pose.position = Point(.4, .4, .5)

        goal_pose.orientation = orientation_from_rpy(pi / 2, -pi / 4, pi)
        resp = self.client.move_ptp(goal_pose)
        self.assert_last_srv_call_success(resp)

    def test_2_move_lin_few_waypoints(self):
        # move into home position
        resp = self.client.move_joints(0, pi / 100, 0, -pi / 2, 0, -pi / 2, 0)
        self.assert_last_srv_call_success(resp)

        goal_pose = Pose()
        goal_pose.position = Point(.3, .41, .62)
        goal_pose.orientation = orientation_from_rpy(-pi / 2, 0, 0)
        resp = self.client.move_ptp(goal_pose)

        # only change the orientation a little, no linear motion -> should fail
        goal_pose.orientation = orientation_from_rpy(-pi / 1.9, 0, 0)
        success = self.client.move_lin(goal_pose)
        self.assert_last_srv_call_failed(
            success, RLLErrorCode.TOO_FEW_WAYPOINTS)

        # only change the translation a little, no linear motion -> should fail
        goal_pose.position = Point(.305, .41, .62)
        success = self.client.move_lin(goal_pose)
        self.assert_last_srv_call_failed(
            success, RLLErrorCode.TOO_FEW_WAYPOINTS)

        # only change the position, orientation from before -> should succeed
        goal_pose.position = Point(.19, .41, .63)
        goal_pose.orientation = orientation_from_rpy(-pi / 2, 0, 0)
        success = self.client.move_lin(goal_pose)
        self.assert_last_srv_call_success(success)

    def test_3_parallel_move_calls(self):
        def do_move_random(_):
            # construct a new service proxy, using the same proxy results in
            # tcp connection issues
            srv = rospy.ServiceProxy('move_random', rll_msgs.srv.MoveRandom)
            resp = srv.call()
            return resp

        results = concurrent_call(do_move_random, count=4,
                                  delay_between_starts=.1)

        # first should have succeeded (it is an recoverable failure)
        self.assert_error_code_equals(results[0].error_code,
                                      RLLErrorCode.SUCCESS)

        for result in results[1:]:
            self.assert_error_code_equals(result.error_code,
                                          RLLErrorCode.CONCURRENT_SERVICE_CALL)

    def test_4_move_lin_collision(self):
        # ensuring same global configuration for IK
        resp = self.client.move_joints(0, -pi / 100, 0, -pi / 2, 0, pi / 2, 0)
        self.assert_last_srv_call_success(resp)
        goal_pose = Pose()
        goal_pose.position = Point(-0.15, .4, .2)
        goal_pose.orientation = orientation_from_rpy(0, pi, 0)
        self.assert_move_ptp_success(goal_pose)

        goal_pose.position.z = .1
        self.assert_move_lin_success(goal_pose)

        goal_pose.position.z = 0.0
        resp = self.client.move_lin(goal_pose)
        self.assert_last_srv_call_failed(resp,
                                         RLLErrorCode.NO_IK_SOLUTION_FOUND)

        goal_pose.position.y = .55
        goal_pose.position.z = .1
        self.assert_move_lin_success(goal_pose)

        goal_pose.position.y = 0.7
        resp = self.client.move_lin(goal_pose)
        self.assert_last_srv_call_failed(resp, RLLErrorCode.GOAL_IN_COLLISION)

    def test_5_move_armangle(self):
        self.client.move_joints(0, pi / 100, 0, -pi / 2, 0, pi / 2, 0)

        goal_pose = Pose()
        goal_pose.position = Point(-0.4, -0.5, .3)
        goal_pose.orientation = orientation_from_rpy(0, pi, 0)
        goal_arm_angle = pi / 2
        resp = self.client.move_ptp_armangle(goal_pose, goal_arm_angle)
        self.assert_last_srv_call_failed(resp,
                                         RLLErrorCode.MOVEIT_PLANNING_FAILED)

        self.assert_move_ptp_success(goal_pose)
        resp = self.client.move_lin_armangle(goal_pose, goal_arm_angle, True)
        self.assert_last_srv_call_failed(
            resp, RLLErrorCode.ONLY_PARTIAL_PATH_PLANNED)

        goal_arm_angle = 3 * pi / 2
        resp = self.client.move_lin_armangle(goal_pose, goal_arm_angle, True)
        self.assert_last_srv_call_failed(resp,
                                         RLLErrorCode.INVALID_INPUT)
        resp = self.client.move_ptp_armangle(goal_pose, goal_arm_angle)
        self.assert_last_srv_call_failed(resp,
                                         RLLErrorCode.INVALID_INPUT)

    def test_6_idle_during_job_run(self):
        # this should result in an internal error, it will also crash
        # the current run_job action
        idle_success = idle()
        self.assertFalse(idle_success)

        self.assertRaises(CriticalServiceCallFailure, self.client.move_random)
