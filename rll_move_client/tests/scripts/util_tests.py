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
import unittest
import numpy as np

from rll_move_client.util import (
    quaternion_to_array,
    orientation_from_rpy
)


class TestUtilFunctions(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestUtilFunctions, self).__init__(*args, **kwargs)

    def test_0_orientation(self):
        ori = orientation_from_rpy(0, 0, 0)
        self.assertTrue(np.allclose(quaternion_to_array(ori), [0, 0, 0, 1]))

        # TODO(mark): add tests
