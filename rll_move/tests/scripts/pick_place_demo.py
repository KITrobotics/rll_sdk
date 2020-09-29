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
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA
from rll_move_client.client import RLLDefaultMoveClient
from rll_move_client.util import orientation_from_rpy

# NOTE: assumes that the move_iface_test interface is running which
# has two grasp objects and three placement locations
from test_iface_util import (grip_pose_at, CYLINDER_HEIGHT, BOX_HEIGHT,
                             INITIAL_GRIP_POSE_CYLINDER, INITIAL_GRIP_POSE_BOX)

GRIP_FAILED_COLOR = ColorRGBA(1, 0, 0, .25)
GRIP_SUCCESS_COLOR = ColorRGBA(0, 1, 0, .25)


def execute(client):
    # type: (RLLDefaultMoveClient) -> bool

    approach_pose1 = grip_pose_at(1, .2)
    approach_pose2 = grip_pose_at(2, .2)
    approach_pose3 = grip_pose_at(3, .2)

    grip_pose_cylinder1 = INITIAL_GRIP_POSE_CYLINDER
    grip_pose_cylinder3 = grip_pose_at(3, CYLINDER_HEIGHT / 2)
    grip_pose_box1 = INITIAL_GRIP_POSE_BOX
    grip_pose_box2 = grip_pose_at(2, BOX_HEIGHT / 2.0)

    # self.validate_pick_place(grip_pose1, True, "box1")
    # self.validate_pick_place(grip_pose1, False, "box1")
    # TODO(mark): this should fail! the cylinder is NOT at this position
    # TODO(mark): add more error codes for failed constraints
    # self.validate_pick_place(grip_pose1, True, "cylinder1")
    # self.validate_pick_place(grip_pose1, False, "cylinder1")
    rospy.loginfo("Moving into position to pick and place box1")
    # Note: move to position with joints to avoid possible PTP issues
    # resp = self.move_ptp(approach_pose3)
    # self.move_joints(joint_values_at_z15(1))

    client.move_ptp(
        Pose(approach_pose3.position, orientation_from_rpy(0, pi, 0)))

    client.pick_place_here(grip_pose_box1, True, "box1")

    client.move_ptp(approach_pose2)
    client.pick_place_here(grip_pose_box2, False, "box1")

    client.move_ptp(approach_pose1)
    client.pick_place_here(grip_pose_cylinder1, True, "cylinder1")
    client.move_ptp(approach_pose2)
    client.pick_place(approach_pose3, grip_pose_cylinder3, approach_pose3,
                      False, "cylinder1")

    return True


def main():
    from rll_tools.run import run_project_in_background

    rospy.init_node("pick_place_demo")
    client = RLLDefaultMoveClient()
    run_project_in_background(1)
    client.spin(oneshot=True)


if __name__ == "__main__":
    main()
