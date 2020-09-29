import math

from geometry_msgs.msg import Pose, Point

from rll_move_client.util import (DOWN, orientation_from_rpy)

# NOTE: assumes that the move_iface_test interface is running which
# has two grasp objects and three placement locations
CYLINDER_HEIGHT = 0.06
CYLINDER_RADIUS = 0.05
BOX_HEIGHT = 0.02
BOX_WIDTH = 0.02
BOX_DEPTH = 0.02

_DROP_LOC1 = (0.4, 0.2, 0)
_DROP_LOC2 = (0.4, -0.25, 0)
_DROP_LOC3 = (0, 0.55, 0)

# Pose at the three drop locations for z=.15
_DROP_LOC1_VALUES = [0.958689, 1.218188, -1.512934, -1.229143, 1.218656,
                     1.635968, -0.271169]

# TODO: [-0.461639,0.873816, -0.142767, -0.93732, 0.111687, .33689, -0.579434]
# these values followed by moveLin(grip_pose_at(.03)) result in only
# partial path planned

_DROP_LOC2_VALUES = [-0.39210418603801633, 0.7458344682596398,
                     -0.00522691295081722, -1.1555284819806646,
                     0.004155307089323994, 1.2394342247929733,
                     -0.39657415735760815]

_DROP_LOC3_VALUES = [-1.5121257178323382, -0.668209647067786,
                     -0.6273856844170336, 1.4163235580350029,
                     -2.7376996392842625, 1.1860984036194375,
                     0.9534245902175177]

_ALL_DROP_VALUES = [_DROP_LOC1_VALUES, _DROP_LOC2_VALUES, _DROP_LOC3_VALUES]

DROP_AREA_1_RADIUS = .025

POS_BOX = Point(_DROP_LOC3[0], _DROP_LOC3[1], BOX_HEIGHT / 2)
POS_CYLINDER = Point(_DROP_LOC1[0], _DROP_LOC1[1], CYLINDER_HEIGHT / 2)


def drop_pos(location, z_pos):
    assert 0 < location < 4
    if location == 1:
        return Point(_DROP_LOC1[0], _DROP_LOC1[1], z_pos)
    elif location == 2:
        return Point(_DROP_LOC2[0], _DROP_LOC2[1], z_pos)

    return Point(_DROP_LOC3[0], _DROP_LOC3[1], z_pos)


def joint_values_at_z15(location):
    assert 0 < location < 4
    return _ALL_DROP_VALUES[location - 1]


def grip_pose_at(location, z_pos=.05):
    return Pose(drop_pos(location, z_pos), DOWN)


INITIAL_GRIP_POSE_CYLINDER = grip_pose_at(1, CYLINDER_HEIGHT / 2)
INITIAL_GRIP_POSE_BOX = Pose(POS_BOX,
                             orientation_from_rpy(0, math.pi, math.pi * 0.25))
