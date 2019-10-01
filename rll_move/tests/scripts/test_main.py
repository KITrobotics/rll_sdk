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

import rospy
from test_util import generate_test_callback, run_project_in
from rll_move_client.client import RLLDefaultMoveClient

from invalid_movements import TestInvalidMovements
from basic_movements import TestBasicMovements
from repeat_movements import TestRepeatedMovements

if __name__ == "__main__":
    tests = [('move_basic', TestBasicMovements),
             ('move_repetition', TestRepeatedMovements),
             ('move_invalid', TestInvalidMovements),
             ]

    execute = generate_test_callback("rll_move", tests)

    # setup a regular move client and run the tests in the execute callback
    rospy.init_node("test_move_iface_client")
    client = RLLDefaultMoveClient(execute)
    run_project_in(10)
    rospy.spin()
