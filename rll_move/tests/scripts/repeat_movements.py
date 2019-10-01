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
from rll_move_client.util import orientation_from_rpy, compare_joint_values

from test_util import TestCaseWithRLLMoveClient
from rll_move_client.error import RLLErrorCode
import rospy


class TestRepeatedMovements(TestCaseWithRLLMoveClient):

    def __init__(self, *args, **kwargs):
        super(TestRepeatedMovements, self).__init__(*args, **kwargs)

    def test_1_repeated_move_lin_reset(self):
        self.repeat_movement(3, True)

    def test_2_repeated_move_lin_reset_failure(self):
        self.repeat_movement(3, True, True)

    def test_3_repeated_move_lin_no_reset(self):
        self.repeat_movement(3, False)

    def test_4_repeated_move_lin_no_reset_failure(self):
        self.repeat_movement(3, False, True)

    def repeat_movement(self, n=10, reset=True, cause_failure=False):
        last_joint_values = None
        for i in range(n):
            rospy.loginfo("Run %d/%d", i + 1, n)

            if reset:
                # reset joints to known start position
                resp = self.client.move_joints(0, 0, 0, -pi / 2, 0, -pi / 2, 0)
                self.assertLastServiceCallSucceeded(resp)

            goal_pose = Pose()
            goal_pose.position = Point(.3, .41, .63)
            goal_pose.orientation = orientation_from_rpy(-pi / 2, 0, 0)

            # move into position
            resp = self.client.move_ptp(goal_pose)
            self.assertLastServiceCallSucceeded(resp)

            if cause_failure:
                # only change the orientation no motion -> should fail
                goal_pose.orientation = orientation_from_rpy(0, 0, 0)
                success = self.client.move_lin(goal_pose)
                self.assertLastServiceCallFailedWith(
                    success, RLLErrorCode.TOO_FEW_WAYPOINTS)

            # only change the position -> should succeed
            goal_pose.position = Point(.2, .41, .63)
            goal_pose.orientation = orientation_from_rpy(-pi / 2, 0, 0)
            success = self.client.move_lin(goal_pose)
            self.assertLastServiceCallSucceeded(success)
            joint_values = self.client.get_current_joint_values()

            if last_joint_values is not None:
                # check if moveit chose the same pose every time
                # this is not a test, just out of curiosity
                if not compare_joint_values(joint_values, last_joint_values):
                    rospy.loginfo("Chose different joint values!")

            last_joint_values = joint_values
