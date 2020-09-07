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

from rll_move_client.error import RLLErrorCode
from rll_move_client.util import orientation_from_rpy, compare_joint_values

from rll_move_client.tests_util import TestCaseWithRLLMoveClient


class TestRepeatedMovements(TestCaseWithRLLMoveClient):

    def test1_move_lin_reset(self):
        self.repeat_movement(3, True)

    def test2_move_lin_reset_fail(self):
        self.repeat_movement(3, True, True)

    def test3_move_lin_no_reset(self):
        self.repeat_movement(3, False)

    def test4_move_lin_no_reset_fail(self):
        self.repeat_movement(3, False, True)

    def repeat_movement(self, count=10, reset=True, cause_failure=False):
        last_joint_values = None
        for i in range(count):
            rospy.loginfo("Run %d/%d", i + 1, count)

            if reset:
                # reset joints to known start position
                resp = self.client.move_joints(0, pi / 100, 0, -pi / 2, 0,
                                               -pi / 2, 0)
                self.assert_last_srv_call_success(resp)

            goal_pose = Pose()
            goal_pose.position = Point(.3, .41, .63)
            goal_pose.orientation = orientation_from_rpy(-pi / 2, 0, 0)

            # move into position
            resp = self.client.move_ptp(goal_pose)
            self.assert_last_srv_call_success(resp)

            if cause_failure:
                # only change the orientation a little,
                # no linear motion -> should fail
                goal_pose.orientation = orientation_from_rpy(-pi / 1.9, 0, 0)
                success = self.client.move_lin(goal_pose)
                self.assert_last_srv_call_failed(
                    success, RLLErrorCode.TOO_FEW_WAYPOINTS)

            # change the position, orientatin stays -> should succeed
            goal_pose.position = Point(.2, .41, .63)
            goal_pose.orientation = orientation_from_rpy(-pi / 2, 0, 0)
            success = self.client.move_lin(goal_pose)
            self.assert_last_srv_call_success(success)
            joint_values = self.client.get_current_joint_values()

            if last_joint_values is not None:
                # check if moveit chose the same pose every time
                # this is not a test, just out of curiosity
                if not compare_joint_values(joint_values, last_joint_values):
                    rospy.loginfo("Chose different joint values!")

            last_joint_values = joint_values
