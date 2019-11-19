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

import numpy

from geometry_msgs.msg import Quaternion
import tf.transformations

# ANSI terminal color codes
C_BOLD = '\033[1m'
C_BLUE = '\033[94m'
C_NAME = C_BLUE + C_BOLD
C_GREEN = '\033[92m'
C_YELLOW = '\033[93m'
C_RED = '\033[91m'
C_MAGENTA = '\033[95m'
C_OK = C_GREEN + C_BOLD
C_INFO = C_YELLOW + C_BOLD
C_WARN = C_MAGENTA + C_BOLD
C_FAIL = C_RED + C_BOLD
C_END = '\033[0m'


def apply_ansi_format(text, ansi_code=C_NAME):
    return "{0}{1}{2}".format(ansi_code, text, C_END)


def array_to_quaternion(arr):
    return Quaternion(arr[0], arr[1], arr[2], arr[3])


def euler_to_quaternion(roll, pitch, yaw, axes='sxyz'):
    """
    Construct a `Quaternion` from the given euler angles

    :param roll:
    :param pitch:
    :param yaw:
    :param axes:
    :return:
    """
    return array_to_quaternion(
        tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes))


def orientation_from_rpy(roll, pitch, yaw):
    return euler_to_quaternion(roll, pitch, yaw, 'sxyz')


def compare_joint_values(joint_values_1, joint_values_2, atol=1.e-4):
    return numpy.allclose(joint_values_1, joint_values_2, atol=atol)
