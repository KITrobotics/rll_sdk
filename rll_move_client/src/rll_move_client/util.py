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
import rospy
import tf.transformations
from geometry_msgs.msg import Quaternion
from rll_move_client.error import RLLErrorCode

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


def euler_to_matrix(alpha, beta, gamma, axes='sxyz'):
    return tf.transformations.quaternion_matrix(
        tf.transformations.quaternion_from_euler(alpha, beta, gamma, axes))


def euler_to_quaternion(alpha, beta, gamma, axes='sxyz'):
    """
    Construct a `Quaternion` from the given euler angles

    :param alpha:
    :param beta:
    :param gamma:
    :param axes:
    :return:
    """
    return array_to_quaternion(
        tf.transformations.quaternion_from_euler(alpha, beta, gamma, axes))


def orientation_from_rpy(roll, pitch, yaw):
    return euler_to_quaternion(roll, pitch, yaw, 'sxyz')


def compare_joint_values(joint_values_1, joint_values_2, atol=1.e-4):
    return numpy.allclose(joint_values_1, joint_values_2, atol=atol)


def compare_poses(pose1, pose2, atol=1.e-4):
    values1 = [pose1.position.x, pose1.position.y, pose1.position.z,
               pose1.orientation.x, pose1.orientation.y, pose1.orientation.z,
               pose1.orientation.w]
    values2 = [pose2.position.x, pose2.position.y, pose2.position.z,
               pose2.orientation.x, pose2.orientation.y, pose2.orientation.z,
               pose2.orientation.w]
    return numpy.allclose(values1, values2, atol=atol)


def register_client_service(client, name, msg_type, callback_func):
    """
    Register a ROS service with the given name and message type. If the service
    is called the callback_func is invoked and its return value treated as the
    response. If the message type has the usual success, error_code values
    the will be set automatically.

    :param client:
    :param name: the service name
    :param msg_type: the service message type
    :param callback_func: the service callback
    :return:
    """

    def callback_wrapper(*args):
        # wrap the callback function to ensure the arguments are passed if
        # required and that the return values are set properly

        if args:
            res = callback_func(client, *args)
        else:
            res = callback_func(client, )

        # get the slots (the fields) of the response message
        # pylint: disable=protected-access
        slots = msg_type._response_class.__slots__

        # ensure its a tuple or list (iterable)
        if not isinstance(res, (list, tuple)):
            res = (res,)

        # in case the amount of return values match, return them directly
        if len(res) == len(slots):
            return res

        resp = []  # build a ordered list of response values
        success = bool(res)  # overall success

        # success and error_code are required values
        expects_default = "error_code" in slots and "success" in slots
        slots_match = len(slots) == 2 or len(slots) == len(res) + 2

        if not expects_default or not slots_match:
            raise ValueError("Not enough values for service call")

        res_index = 0
        # iterate over the expected slots and fill as best as possible since
        # we cannot be sure in what order the arguments are -> better iterate
        for slot in slots:
            if slot == "success":
                resp.append(success)
            elif slot == "error_code":
                code = RLLErrorCode.SUCCESS if success else \
                    RLLErrorCode.RECOVERABLE_FAILURE
                resp.append(code)
            else:
                if not success:
                    resp.append(None)  # fill with None on failure
                else:
                    resp.append(res[res_index])

        return resp

    service = rospy.Service(name, msg_type, callback_wrapper)
    return service
