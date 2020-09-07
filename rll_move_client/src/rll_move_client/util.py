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
from math import sqrt, pi
from typing import (List, Iterable, Sequence, Dict, Union)  # noqa: E501, pylint: disable=unused-import, line-too-long


import numpy as np
import rospy
from geometry_msgs.msg import Quaternion, Pose, Point
from tf.transformations import (unit_vector, quaternion_from_euler,
                                quaternion_matrix, euler_from_quaternion,
                                quaternion_from_matrix,
                                quaternion_multiply)
from rll_move_client.error import RLLErrorCode, ServiceCallFailure

# custom type to make the type hints more readable
PointLike = Union[Sequence[float], Point, Pose, np.ndarray]  # noqa: E501, pylint: disable=invalid-name, line-too-long


def array_to_point(arr):
    # type: (Sequence[float]) -> Point
    return Point(*arr[:3])


def point_to_array(point):
    # type: (Point) -> np.ndarray
    return np.array([point.x, point.y, point.z])


def array_to_quaternion(arr):
    # type: (Sequence[float]) -> Quaternion
    return Quaternion(arr[0], arr[1], arr[2], arr[3])


def quaternion_to_array(quaternion):
    # type: (Quaternion) -> List[float]
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]


def quaternion_to_rpy(quaternion):
    # type: (Quaternion) -> List[float, float, float]

    return euler_from_quaternion(quaternion_to_array(quaternion))


def euler_to_matrix(alpha, beta, gamma, axes='sxyz'):
    # type: (float, float, float, str) -> np.ndarray
    return quaternion_matrix(quaternion_from_euler(alpha, beta, gamma, axes))


def matrix_to_pose(matrix):
    # type: (np.ndarray) -> Pose
    quat_array = quaternion_from_matrix(matrix)
    orientation = array_to_quaternion(quat_array)
    position = array_to_point(matrix[0:3, 3])

    return Pose(position=position, orientation=orientation)


def pose_to_matrix(pose):
    # type: (Pose) -> np.ndarray

    np_matrix = quaternion_matrix([pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w])

    np_matrix[0][3] = pose.position.x
    np_matrix[1][3] = pose.position.y
    np_matrix[2][3] = pose.position.z

    return np_matrix


def matrix_to_quaternion(matrix):
    # type: (np.ndarray) -> Quaternion
    quat_array = quaternion_from_matrix(matrix)
    orientation = array_to_quaternion(quat_array)

    return orientation


def construct_transform_matrix(point, x_dir, y_dir, z_dir):
    # type: (PointLike, Sequence[float], Sequence[float], Sequence[float]) -> np.ndarray # noqa: E501, pylint: disable=line-too-long
    pos = pointlike_to_array(point)
    return np.array([
        [x_dir[0], y_dir[0], z_dir[0], pos[0]],
        [x_dir[1], y_dir[1], z_dir[1], pos[1]],
        [x_dir[2], y_dir[2], z_dir[2], pos[2]],
        [0, 0, 0, 1]
    ])


def pointlike_to_array(point):
    # type: (PointLike) -> Sequence[float]

    if isinstance(point, Point):
        return [point.x, point.y, point.z]
    elif isinstance(point, Pose):
        return [point.position.x, point.position.y, point.position.z]
    elif isinstance(point, (list, tuple)):
        if len(point) < 3:
            raise ValueError("%s is not a valid point." % point)
        return point[:3]
    elif isinstance(point, np.ndarray):
        if len(point.shape) != 1 or point.shape[0] < 3:
            raise ValueError("%s is not a valid point." % point)
        return point[:3]

    raise ValueError("Invalid (non point) data: %s" % point)


def apply_transform4x4_point3(transform, point):
    # type: (np.ndarray, PointLike) -> np.ndarray

    array = pointlike_to_array(point)
    return (np.dot(transform, [array[0], array[1], array[2], 1]))[:3]


def direction_from_quaternion(orientation, axis=2):
    # type: (Quaternion, int) -> np.ndarray

    # 'z-axis' of the rotation matrix represents the 'forward' axis
    matrix = quaternion_matrix(
        quaternion_to_array(orientation))

    axis = max(0, min(2, axis))
    direction = matrix[0:3, axis]
    return unit_vector(direction)


def translate_pose_relative(pose, t_x=0, t_y=0, t_z=0):
    # type: (Pose, float, float, float) -> Pose
    """
    Returns a pose by translating the given pose along its relative coordinate
    axes by the given factors.
    """
    _pose = Pose(Point(pose.position.x, pose.position.y, pose.position.z),
                 Quaternion(pose.orientation.x, pose.orientation.y,
                            pose.orientation.z, pose.orientation.w))
    matrix = quaternion_matrix(
        quaternion_to_array(pose.orientation))

    x_dir = unit_vector(matrix[0:3, 0])
    y_dir = unit_vector(matrix[0:3, 1])
    z_dir = unit_vector(matrix[0:3, 2])

    _pose.position.x += t_x * x_dir[0] + t_y * y_dir[0] + t_z * z_dir[0]
    _pose.position.y += t_x * x_dir[1] + t_y * y_dir[1] + t_z * z_dir[1]
    _pose.position.z += t_x * x_dir[2] + t_y * y_dir[2] + t_z * z_dir[2]

    return _pose


def pose_point_at(position, z_axis, rough_x_axis):
    # type: (PointLike, PointLike, PointLike) -> Pose

    tmp_x_dir = unit_vector(pointlike_to_array(rough_x_axis))
    z_dir = unit_vector(pointlike_to_array(z_axis))
    y_dir = unit_vector(np.cross(z_dir, tmp_x_dir))
    x_dir = unit_vector(np.cross(y_dir, z_dir))

    matrix = construct_transform_matrix(position, x_dir, y_dir, z_dir)
    return matrix_to_pose(matrix)


def offset_point(point, o_x=0, o_y=0, o_z=0):
    # type: (Point, float, float, float) -> Point
    return Point(point.x + o_x, point.y + o_y, point.z + o_z)


def translate_point(point, translate_by_or_x, t_y=0, t_z=0, scale=1):
    # type: (PointLike, Union[PointLike, float], float, float, float) -> Point

    if isinstance(translate_by_or_x, float):
        t_x = translate_by_or_x
    else:
        t_x, t_y, t_z = pointlike_to_array(translate_by_or_x)

    p_x, p_y, p_z = pointlike_to_array(point)
    return Point(p_x + t_x * scale, p_y + t_y * scale, p_z + t_z * scale)


def linear_combine_points(point1, point2, scale1=1.0, scale2=1.0):
    # type: (PointLike, PointLike, float, float) -> Sequence[float]
    """
    Combines the two given points like objects componentwise  according to:
    x = scale1 * point1 + scale2 * point2
    """
    p_1 = pointlike_to_array(point1)
    p_2 = pointlike_to_array(point2)
    return [p_1[i] * scale1 + p_2[i] * scale2 for i in range(3)]


def euler_to_quaternion(alpha, beta, gamma, axes='sxyz'):
    # type: (float, float, float, str) -> Quaternion
    """
    Construct a `Quaternion` from the given euler angles
    """

    quaternion = quaternion_from_euler(alpha, beta, gamma, axes)
    magnitude = np.linalg.norm(quaternion)
    return array_to_quaternion(quaternion / magnitude)


def rotate_quaternion(quat, roll, pitch, yaw):
    # type: (Quaternion, float, float, float) -> Quaternion

    orig_tf_quat = quaternion_to_array(quat)
    tf_quat = quaternion_from_euler(roll, pitch, yaw)
    rotated_quat = quaternion_multiply(orig_tf_quat, tf_quat)
    return array_to_quaternion(rotated_quat)


def orientation_from_rpy(roll, pitch, yaw):
    # type: (float, float, float) -> Quaternion

    return euler_to_quaternion(roll, pitch, yaw, 'sxyz')


def pose2d_to_3d(pos_x, pos_y, theta, pos_z=0):
    return Pose(Point(pos_x, pos_y, pos_z), orientation_from_rpy(0, 0, theta))


def distance_between(point1, point2):
    # type: (PointLike, PointLike) -> float

    x_1, y_1, z_1 = point_to_array(point1)
    x_2, y_2, z_2 = point_to_array(point2)

    distance = sqrt((x_1 - x_2) ** 2 + (y_1 - y_2) ** 2 + (z_1 - z_2) ** 2)
    return distance


def compare_joint_values(joint_values_1, joint_values_2, atol=1.e-4):
    # type: (Iterable[float], Iterable[float], float) -> bool

    return np.allclose(joint_values_1, joint_values_2, atol=atol)


def compare_poses(pose1, pose2, atol_pos=1.e-2, atol_rot=1.e-2):
    # type: (Pose, Pose, float, float) -> bool

    if not isinstance(pose1, Pose) or not isinstance(pose2, Pose):
        rospy.logwarn("Cannot compare non-poses: %s and %s", pose1, pose2)
        return False

    cmp_pos = compare_points(pose1.position, pose2.position, atol_pos)
    cmp_rot = compare_orientation(pose1.orientation, pose2.orientation,
                                  atol_rot)

    return cmp_pos and cmp_rot


def compare_orientation(ori1, ori2, atol_rot=1.e-2):
    # type: (Quaternion, Quaternion, float) -> bool

    values1_rot = quaternion_to_rpy(ori1)
    values2_rot = quaternion_to_rpy(ori2)
    cmp_rot = np.allclose(values1_rot, values2_rot, atol=atol_rot)

    return cmp_rot


def compare_list_of_points(actual_points, expected_points):
    # type: (List[Point], List[Point]) -> bool

    copy = list(actual_points)
    if len(actual_points) != len(expected_points):
        return False

    for point in expected_points:
        found_at = -1
        for i, point_2 in enumerate(copy):
            if compare_points(point, point_2):
                found_at = i
                break
        if found_at == -1:
            return False

        del copy[found_at]

    return True


def compare_points(point1, point2, atol_pos=1.e-2):
    # type: (Point, Point, float) -> bool
    values1_pos = [point1.x, point1.y, point1.z]
    values2_pos = [point2.x, point2.y, point2.z]

    cmp_pos = compare_lists(values1_pos, values2_pos, atol_pos)
    return cmp_pos


def compare_lists(values1_pos, values2_pos, atol_pos=1.e-2):
    # cast to float, because np doesn't like mixed int and float types
    val1 = list(map(float, values1_pos))
    val2 = list(map(float, values2_pos))
    cmp_pos = np.allclose(val1, val2, atol=atol_pos)
    return cmp_pos


_REGISTERED_SERVICES = {}  # type: Dict[str, rospy.Service]


def register_client_service(client, name, msg_type,  # noqa: C901, E501
                            callback_func):
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

        try:
            if args:
                res = callback_func(client, *args)
            else:
                res = callback_func(client, )
        except NotImplementedError as expt:
            rospy.logwarn("Custom service %s raised not implemented error: %s",
                          name, expt.message)
            res = None
        except Exception as expt:
            # trace = "".join(traceback.format_stack()[:-1])
            raise ServiceCallFailure(
                RLLErrorCode.SERVICE_CALL_CLIENT_ERROR,
                "Your service function %s raised an exception: \n%s" % (
                    name, expt))

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

    if name in _REGISTERED_SERVICES:
        rospy.logwarn("Service with name %s, already registered!", name)
        return None

    service = rospy.Service(name, msg_type, callback_wrapper)
    _REGISTERED_SERVICES[name] = service
    return service


UP = Quaternion(0, 0, 0, 1)
DOWN = orientation_from_rpy(0, pi, 0)
LEFT = orientation_from_rpy(0, pi / 2, pi / 2)
RIGHT = orientation_from_rpy(0, pi / 2, -pi / 2)
