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
from rll_move_client.util import orientation_from_rpy

from rll_move_client.test import TestCaseWithRLLMoveClient
from rll_move_client.error import RLLErrorCode


class TestInvalidMovements(TestCaseWithRLLMoveClient):

    def __init__(self, *args, **kwargs):
        super(TestInvalidMovements, self).__init__(*args, **kwargs)

    def test_1_pose_outside_workspace(self):
        goal_pose = Pose()
        goal_pose.position = Point(1, 1, 1)
        resp = self.client.move_ptp(goal_pose)
        self.assertServiceCallFailedWith(resp, RLLErrorCode.NO_IK_SOLUTION_FOUND)

    def parallel_move_calls(self, client):
        import threading

        def try_move_random():
            resp = client.move_random()
            assert resp

        threads = [threading.Thread(target=try_move_random) for _ in range(5)]
        for thread in threads:
            thread.start()

        for thread in threads:
            thread.join()

    def test_1_move_ptp(self):
        goal_pose = Pose()
        goal_pose.position = Point(.4, .4, .5)

        # move to starting position
        goal_pose.orientation = orientation_from_rpy(pi / 2, -pi / 4, pi)
        resp = self.client.move_ptp(goal_pose)
        self.assertTrue(resp, "failed to move_ptp")

    def test_2_move_lin_sole_rotation(self):
        goal_pose = Pose()
        goal_pose.position = Point(.4, .4, .5)

        # only change the orientation no motion -> should fail
        goal_pose.orientation = orientation_from_rpy(pi / 2, -pi / 2, pi)
        success = self.client.move_lin(goal_pose)
        self.assertServiceCallFailedWith(success, RLLErrorCode.TOO_FEW_WAYPOINTS)

        # only change the position -> should succeed
        goal_pose.position = Point(.4, .2, .5)
        goal_pose.orientation = orientation_from_rpy(pi / 2, -pi / 4, pi)
        success = self.client.move_lin(goal_pose)
        self.assertServiceCallSuccess(success)
