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
from geometry_msgs.msg import Pose, Point

from rll_move_client.client import RLLDefaultMoveClient
from rll_move_client.util import orientation_from_rpy, compare_joint_values


def execute(move_client):
    # demonstrates how to use the available services:
    #
    # 1. moveJoints(a1, a2, ..., a7) vs move_joints(joint_values)
    # 2. move_ptp(pose)
    # 3. move_lin(pose)
    # 4. generated_pose = move_random()
    # 5. pose = get_current_pose()
    # 6. joint_values = get_current_joint_values()

    resp = move_client.move_joints(0.0, 0.0, 0.0, -pi / 2, 0.0, 0.0, 0.0)
    # returns True/False to indicate success, throws of critical failure
    if not resp:
        rospy.logerr("move_joints service call failed")

    joint_values = [pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    move_client.move_joints(joint_values)

    goal_pose = Pose()
    goal_pose.position = Point(.4, .4, .5)
    goal_pose.orientation = orientation_from_rpy(pi / 2, -pi / 4, pi)

    # move ptp to the specified pose
    move_client.move_ptp(goal_pose)

    goal_pose.position.x = 0.2
    goal_pose.position.y = .5
    # linear movement to the new pose
    move_client.move_lin(goal_pose)

    # move to random pose, returns Pose/None to indicate success
    resp = move_client.move_random()

    if resp:
        # response contains the randomly generated pose
        rospy.loginfo("move_random moved to: %s", resp)

    # get current pose, should match the previous random pose
    pose = move_client.get_current_pose()
    rospy.loginfo("current end effector pose: %s", pose)

    # set the joint values again
    joint_values2 = [pi / 2, 0.2, 0, 0, -pi / 4, .24, 0]
    move_client.move_joints(joint_values2)

    # retrieve the previously set joint values
    joint_values = move_client.get_current_joint_values()

    match = compare_joint_values(joint_values, joint_values2)
    rospy.loginfo("Set and queried joint values match: %s",
                  "yes" if match else "no")


def main():
    rospy.init_node('move_client_example')
    client = RLLDefaultMoveClient(execute)
    client.spin()


if __name__ == "__main__":
    main()
