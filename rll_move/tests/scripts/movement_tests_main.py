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

import time
import rospy
from rll_move_client.client import RLLDefaultMoveClient
from rll_move_client.tests_util import generate_test_callback, shutdown, \
    TestData
from rll_tools.run import run_project_in_background

from invalid_movements import TestInvalidMovements
from basic_movements import TestBasicMovements
from repeat_movements import TestRepeatedMovements
from before_project_run import TestBeforeProjectRun


def main():
    tests = [TestData('move_basic', TestBasicMovements),
             TestData('move_repetition', TestRepeatedMovements),
             TestData('move_invalid', TestInvalidMovements)]

    tests_before = TestData('move_before', TestBeforeProjectRun)

    execute_before = generate_test_callback("rll_move_before", tests_before,
                                            shutdown_func=None)
    execute = generate_test_callback("rll_move", tests, shutdown_func=shutdown)

    # wait for the move_iface to start and previous clients to exit
    time.sleep(8)

    # setup a regular move client and run the tests in the execute callback
    rospy.init_node("test_move_iface_client")
    client = RLLDefaultMoveClient(execute)

    execute_before(client)
    run_project_in_background(2)
    client.spin(oneshot=True)


if __name__ == "__main__":
    main()
