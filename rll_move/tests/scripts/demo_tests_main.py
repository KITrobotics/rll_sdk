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
import rosunit
from rll_move_client.tests_demo_launcher import create_test_class


def main():
    rospy.init_node("demo_tests")

    # Run the move_client_example demo TODO(mark): put in rll_move_client
    to_import = [("rll_move", "tests/scripts/"), ("rll_move_client", "src")]
    klass = create_test_class(to_import, "move_client_demo", "execute",
                              "rll_move_client.client", "RLLDefaultMoveClient")
    # TODO(wolfgang): variant with test coverage, but returns error
    # because the package is not installed
    # rosunit.unitrun("rll_move", "demo_tests", klass, sysargs="--cov",
    #                 coverage_packages=["rll_move_client"])
    rosunit.unitrun("rll_move", "demo_tests", klass)

    klass = create_test_class(to_import, "pick_place_demo", "execute",
                              "rll_move_client.client", "RLLDefaultMoveClient")

    # TODO(wolfgang): variant with test coverage, but returns error
    # because the package is not installed
    # rosunit.unitrun("rll_move", "pick_place_demo", klass, sysargs="--cov",
    #                 coverage_packages=["rll_move_client"])
    rosunit.unitrun("rll_move", "pick_place_demo", klass)


if __name__ == "__main__":
    main()
