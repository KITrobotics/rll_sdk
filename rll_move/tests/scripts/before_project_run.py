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

import rospy
from test_util import TestCaseWithRLLMoveClient
from rll_move_client.error import RLLErrorCode
from std_srvs.srv import Trigger, TriggerRequest

class TestBeforeProjectRun(TestCaseWithRLLMoveClient):

    def __init__(self, *args, **kwargs):
        super(TestBeforeProjectRun, self).__init__(*args, **kwargs)
        self.robot_ready_service = rospy.ServiceProxy("robot_ready", Trigger)

    def test_0_call_service(self):
        resp = self.client.move_random()
        self.assertLastServiceCallFailedWith(
            resp, RLLErrorCode.SERVICE_CALL_NOT_ALLOWED)


        # robot ready should be allowed even ouside project run
        resp = self.robot_ready_service.call(TriggerRequest())
        self.assertTrue(resp)
        
       
